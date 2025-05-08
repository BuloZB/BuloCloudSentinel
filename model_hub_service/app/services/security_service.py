"""
Security service for the Model Hub.

This module provides a service for securing models and deployments.
"""

import os
import logging
import subprocess
import tempfile
import hashlib
from typing import Dict, List, Any, Optional, BinaryIO, Tuple
from pathlib import Path

# Setup logging
logger = logging.getLogger(__name__)

class SecurityService:
    """Service for securing models and deployments."""
    
    def __init__(self):
        """Initialize the security service."""
        # Check if cosign is available
        self.cosign_available = self._check_cosign()
        
        # Get signing key from environment variables
        self.signing_key = os.environ.get("COSIGN_PRIVATE_KEY")
        self.signing_key_password = os.environ.get("COSIGN_PASSWORD")
        
        # Check if signing is enabled
        self.signing_enabled = self.cosign_available and self.signing_key is not None
        
        if self.signing_enabled:
            logger.info("Model signing is enabled")
        else:
            logger.warning("Model signing is disabled")
    
    def _check_cosign(self) -> bool:
        """
        Check if cosign is available.
        
        Returns:
            True if cosign is available, False otherwise
        """
        try:
            result = subprocess.run(
                ["cosign", "version"],
                capture_output=True,
                text=True,
                check=False,
            )
            
            if result.returncode == 0:
                logger.info(f"Cosign is available: {result.stdout.strip()}")
                return True
            else:
                logger.warning(f"Cosign command failed: {result.stderr.strip()}")
                return False
        except FileNotFoundError:
            logger.warning("Cosign not found in PATH")
            return False
        except Exception as e:
            logger.warning(f"Error checking cosign: {e}")
            return False
    
    async def sign_model(self, model_path: str) -> Optional[str]:
        """
        Sign a model using cosign.
        
        Args:
            model_path: Path to the model file
            
        Returns:
            Signature if signing is successful, None otherwise
        """
        if not self.signing_enabled:
            logger.warning("Model signing is disabled")
            return None
        
        try:
            # Create temporary file for signature
            with tempfile.NamedTemporaryFile(delete=False) as temp_sig_file:
                sig_path = temp_sig_file.name
            
            # Create temporary file for signing key
            with tempfile.NamedTemporaryFile(delete=False, mode="w") as temp_key_file:
                temp_key_file.write(self.signing_key)
                key_path = temp_key_file.name
            
            try:
                # Sign the model
                env = os.environ.copy()
                env["COSIGN_PASSWORD"] = self.signing_key_password
                
                result = subprocess.run(
                    [
                        "cosign", "sign-blob",
                        "--key", key_path,
                        "--output-signature", sig_path,
                        model_path,
                    ],
                    env=env,
                    capture_output=True,
                    text=True,
                    check=True,
                )
                
                # Read signature
                with open(sig_path, "r") as f:
                    signature = f.read().strip()
                
                logger.info(f"Model signed successfully: {model_path}")
                
                return signature
            finally:
                # Clean up temporary files
                if os.path.exists(sig_path):
                    os.unlink(sig_path)
                if os.path.exists(key_path):
                    os.unlink(key_path)
        except subprocess.CalledProcessError as e:
            logger.error(f"Error signing model: {e.stderr}")
            return None
        except Exception as e:
            logger.error(f"Error signing model: {e}")
            return None
    
    async def verify_model(self, model_path: str, signature: str) -> bool:
        """
        Verify a model signature using cosign.
        
        Args:
            model_path: Path to the model file
            signature: Signature to verify
            
        Returns:
            True if verification is successful, False otherwise
        """
        if not self.cosign_available:
            logger.warning("Cosign not available, skipping verification")
            return True
        
        try:
            # Create temporary file for signature
            with tempfile.NamedTemporaryFile(delete=False, mode="w") as temp_sig_file:
                temp_sig_file.write(signature)
                sig_path = temp_sig_file.name
            
            try:
                # Verify the model
                result = subprocess.run(
                    [
                        "cosign", "verify-blob",
                        "--key", "cosign.pub",
                        "--signature", sig_path,
                        model_path,
                    ],
                    capture_output=True,
                    text=True,
                    check=False,
                )
                
                if result.returncode == 0:
                    logger.info(f"Model signature verified: {model_path}")
                    return True
                else:
                    logger.warning(f"Model signature verification failed: {result.stderr}")
                    return False
            finally:
                # Clean up temporary file
                if os.path.exists(sig_path):
                    os.unlink(sig_path)
        except Exception as e:
            logger.error(f"Error verifying model signature: {e}")
            return False
    
    async def calculate_hash(self, model_path: str) -> str:
        """
        Calculate SHA-256 hash of a model file.
        
        Args:
            model_path: Path to the model file
            
        Returns:
            SHA-256 hash of the model file
        """
        try:
            sha256_hash = hashlib.sha256()
            
            with open(model_path, "rb") as f:
                # Read and update hash in chunks
                for byte_block in iter(lambda: f.read(4096), b""):
                    sha256_hash.update(byte_block)
            
            return sha256_hash.hexdigest()
        except Exception as e:
            logger.error(f"Error calculating model hash: {e}")
            raise
    
    async def verify_hash(self, model_path: str, expected_hash: str) -> bool:
        """
        Verify the hash of a model file.
        
        Args:
            model_path: Path to the model file
            expected_hash: Expected SHA-256 hash
            
        Returns:
            True if the hash matches, False otherwise
        """
        try:
            actual_hash = await self.calculate_hash(model_path)
            
            if actual_hash == expected_hash:
                logger.info(f"Model hash verified: {model_path}")
                return True
            else:
                logger.warning(f"Model hash verification failed: {model_path}")
                logger.warning(f"Expected: {expected_hash}")
                logger.warning(f"Actual: {actual_hash}")
                return False
        except Exception as e:
            logger.error(f"Error verifying model hash: {e}")
            return False
    
    async def create_seccomp_profile(self, model_path: str, output_path: str) -> bool:
        """
        Create a seccomp profile for a model.
        
        Args:
            model_path: Path to the model file
            output_path: Path to save the seccomp profile
            
        Returns:
            True if successful, False otherwise
        """
        try:
            # Create a basic seccomp profile
            profile = {
                "defaultAction": "SCMP_ACT_ERRNO",
                "architectures": [
                    "SCMP_ARCH_X86_64",
                    "SCMP_ARCH_X86",
                    "SCMP_ARCH_AARCH64",
                ],
                "syscalls": [
                    {
                        "names": [
                            "read",
                            "write",
                            "open",
                            "close",
                            "stat",
                            "fstat",
                            "lstat",
                            "poll",
                            "lseek",
                            "mmap",
                            "mprotect",
                            "munmap",
                            "brk",
                            "rt_sigaction",
                            "rt_sigprocmask",
                            "rt_sigreturn",
                            "ioctl",
                            "pread64",
                            "pwrite64",
                            "readv",
                            "writev",
                            "access",
                            "pipe",
                            "select",
                            "sched_yield",
                            "mremap",
                            "msync",
                            "mincore",
                            "madvise",
                            "shmget",
                            "shmat",
                            "shmctl",
                            "dup",
                            "dup2",
                            "pause",
                            "nanosleep",
                            "getitimer",
                            "alarm",
                            "setitimer",
                            "getpid",
                            "sendfile",
                            "socket",
                            "connect",
                            "accept",
                            "sendto",
                            "recvfrom",
                            "sendmsg",
                            "recvmsg",
                            "shutdown",
                            "bind",
                            "listen",
                            "getsockname",
                            "getpeername",
                            "socketpair",
                            "setsockopt",
                            "getsockopt",
                            "clone",
                            "fork",
                            "vfork",
                            "execve",
                            "exit",
                            "wait4",
                            "kill",
                            "uname",
                            "semget",
                            "semop",
                            "semctl",
                            "shmdt",
                            "msgget",
                            "msgsnd",
                            "msgrcv",
                            "msgctl",
                            "fcntl",
                            "flock",
                            "fsync",
                            "fdatasync",
                            "truncate",
                            "ftruncate",
                            "getdents",
                            "getcwd",
                            "chdir",
                            "fchdir",
                            "rename",
                            "mkdir",
                            "rmdir",
                            "creat",
                            "link",
                            "unlink",
                            "symlink",
                            "readlink",
                            "chmod",
                            "fchmod",
                            "chown",
                            "fchown",
                            "lchown",
                            "umask",
                            "gettimeofday",
                            "getrlimit",
                            "getrusage",
                            "sysinfo",
                            "times",
                            "ptrace",
                            "getuid",
                            "syslog",
                            "getgid",
                            "setuid",
                            "setgid",
                            "geteuid",
                            "getegid",
                            "setpgid",
                            "getppid",
                            "getpgrp",
                            "setsid",
                            "setreuid",
                            "setregid",
                            "getgroups",
                            "setgroups",
                            "setresuid",
                            "getresuid",
                            "setresgid",
                            "getresgid",
                            "getpgid",
                            "setfsuid",
                            "setfsgid",
                            "getsid",
                            "capget",
                            "capset",
                            "rt_sigpending",
                            "rt_sigtimedwait",
                            "rt_sigqueueinfo",
                            "rt_sigsuspend",
                            "sigaltstack",
                            "utime",
                            "mknod",
                            "uselib",
                            "personality",
                            "ustat",
                            "statfs",
                            "fstatfs",
                            "sysfs",
                            "getpriority",
                            "setpriority",
                            "sched_setparam",
                            "sched_getparam",
                            "sched_setscheduler",
                            "sched_getscheduler",
                            "sched_get_priority_max",
                            "sched_get_priority_min",
                            "sched_rr_get_interval",
                            "mlock",
                            "munlock",
                            "mlockall",
                            "munlockall",
                            "vhangup",
                            "modify_ldt",
                            "pivot_root",
                            "_sysctl",
                            "prctl",
                            "arch_prctl",
                            "adjtimex",
                            "setrlimit",
                            "chroot",
                            "sync",
                            "acct",
                            "settimeofday",
                            "mount",
                            "umount2",
                            "swapon",
                            "swapoff",
                            "reboot",
                            "sethostname",
                            "setdomainname",
                            "iopl",
                            "ioperm",
                            "create_module",
                            "init_module",
                            "delete_module",
                            "get_kernel_syms",
                            "query_module",
                            "quotactl",
                            "nfsservctl",
                            "getpmsg",
                            "putpmsg",
                            "afs_syscall",
                            "tuxcall",
                            "security",
                            "gettid",
                            "readahead",
                            "setxattr",
                            "lsetxattr",
                            "fsetxattr",
                            "getxattr",
                            "lgetxattr",
                            "fgetxattr",
                            "listxattr",
                            "llistxattr",
                            "flistxattr",
                            "removexattr",
                            "lremovexattr",
                            "fremovexattr",
                            "tkill",
                            "time",
                            "futex",
                            "sched_setaffinity",
                            "sched_getaffinity",
                            "set_thread_area",
                            "io_setup",
                            "io_destroy",
                            "io_getevents",
                            "io_submit",
                            "io_cancel",
                            "get_thread_area",
                            "lookup_dcookie",
                            "epoll_create",
                            "epoll_ctl_old",
                            "epoll_wait_old",
                            "remap_file_pages",
                            "getdents64",
                            "set_tid_address",
                            "restart_syscall",
                            "semtimedop",
                            "fadvise64",
                            "timer_create",
                            "timer_settime",
                            "timer_gettime",
                            "timer_getoverrun",
                            "timer_delete",
                            "clock_settime",
                            "clock_gettime",
                            "clock_getres",
                            "clock_nanosleep",
                            "exit_group",
                            "epoll_wait",
                            "epoll_ctl",
                            "tgkill",
                            "utimes",
                            "vserver",
                            "mbind",
                            "set_mempolicy",
                            "get_mempolicy",
                            "mq_open",
                            "mq_unlink",
                            "mq_timedsend",
                            "mq_timedreceive",
                            "mq_notify",
                            "mq_getsetattr",
                            "kexec_load",
                            "waitid",
                            "add_key",
                            "request_key",
                            "keyctl",
                            "ioprio_set",
                            "ioprio_get",
                            "inotify_init",
                            "inotify_add_watch",
                            "inotify_rm_watch",
                            "migrate_pages",
                            "openat",
                            "mkdirat",
                            "mknodat",
                            "fchownat",
                            "futimesat",
                            "newfstatat",
                            "unlinkat",
                            "renameat",
                            "linkat",
                            "symlinkat",
                            "readlinkat",
                            "fchmodat",
                            "faccessat",
                            "pselect6",
                            "ppoll",
                            "unshare",
                            "set_robust_list",
                            "get_robust_list",
                            "splice",
                            "tee",
                            "sync_file_range",
                            "vmsplice",
                            "move_pages",
                            "utimensat",
                            "epoll_pwait",
                            "signalfd",
                            "timerfd_create",
                            "eventfd",
                            "fallocate",
                            "timerfd_settime",
                            "timerfd_gettime",
                            "accept4",
                            "signalfd4",
                            "eventfd2",
                            "epoll_create1",
                            "dup3",
                            "pipe2",
                            "inotify_init1",
                            "preadv",
                            "pwritev",
                            "rt_tgsigqueueinfo",
                            "perf_event_open",
                            "recvmmsg",
                            "fanotify_init",
                            "fanotify_mark",
                            "prlimit64",
                            "name_to_handle_at",
                            "open_by_handle_at",
                            "clock_adjtime",
                            "syncfs",
                            "sendmmsg",
                            "setns",
                            "getcpu",
                            "process_vm_readv",
                            "process_vm_writev",
                            "kcmp",
                            "finit_module",
                            "sched_setattr",
                            "sched_getattr",
                            "renameat2",
                            "seccomp",
                            "getrandom",
                            "memfd_create",
                            "kexec_file_load",
                            "bpf",
                            "execveat",
                            "userfaultfd",
                            "membarrier",
                            "mlock2",
                            "copy_file_range",
                            "preadv2",
                            "pwritev2",
                            "pkey_mprotect",
                            "pkey_alloc",
                            "pkey_free",
                            "statx",
                            "io_pgetevents",
                            "rseq",
                        ],
                        "action": "SCMP_ACT_ALLOW",
                    },
                ],
            }
            
            # Write profile to file
            with open(output_path, "w") as f:
                import json
                json.dump(profile, f, indent=2)
            
            logger.info(f"Created seccomp profile: {output_path}")
            
            return True
        except Exception as e:
            logger.error(f"Error creating seccomp profile: {e}")
            return False
