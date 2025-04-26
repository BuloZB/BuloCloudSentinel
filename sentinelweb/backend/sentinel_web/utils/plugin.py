import os
import re
import subprocess
import sys
from importlib import util
import types
import tempfile
import logging

from sentinel_web.env import SRC_LOG_LEVELS, PIP_OPTIONS, PIP_PACKAGE_INDEX_OPTIONS
from sentinel_web.models.functions import Functions
from sentinel_web.models.tools import Tools

log = logging.getLogger(__name__)
log.setLevel(SRC_LOG_LEVELS["MAIN"])


def extract_frontmatter(content):
    """
    Extract frontmatter as a dictionary from the provided content string.
    """
    frontmatter = {}
    frontmatter_started = False
    frontmatter_ended = False
    frontmatter_pattern = re.compile(r"^\s*([a-z_]+):\s*(.*)\s*$", re.IGNORECASE)

    try:
        lines = content.splitlines()
        if len(lines) < 1 or lines[0].strip() != '"""':
            # The content doesn't start with triple quotes
            return {}

        frontmatter_started = True

        for line in lines[1:]:
            if '"""' in line:
                if frontmatter_started:
                    frontmatter_ended = True
                    break

            if frontmatter_started and not frontmatter_ended:
                match = frontmatter_pattern.match(line)
                if match:
                    key, value = match.groups()
                    frontmatter[key.strip()] = value.strip()

    except Exception as e:
        log.exception(f"Failed to extract frontmatter: {e}")
        return {}

    return frontmatter


def replace_imports(content):
    """
    Replace the import paths in the content.
    """
    replacements = {
        "from utils": "from sentinel_web.utils",
        "from apps": "from sentinel_web.apps",
        "from main": "from sentinel_web.main",
        "from config": "from sentinel_web.config",
    }

    for old, new in replacements.items():
        content = content.replace(old, new)

    return content


def load_tool_module_by_id(tool_id, content=None):

    if content is None:
        tool = Tools.get_tool_by_id(tool_id)
        if not tool:
            raise Exception(f"Toolkit not found: {tool_id}")

        content = tool.content

        content = replace_imports(content)
        Tools.update_tool_by_id(tool_id, {"content": content})
    else:
        frontmatter = extract_frontmatter(content)
        # Install required packages found within the frontmatter
        install_frontmatter_requirements(frontmatter.get("requirements", ""))

    module_name = f"tool_{tool_id}"
    module = types.ModuleType(module_name)
    sys.modules[module_name] = module

    # Create a temporary file and use it to define `__file__` so
    # that it works as expected from the module's perspective.
    temp_file = tempfile.NamedTemporaryFile(delete=False)
    temp_file.close()
    try:
        with open(temp_file.name, "w", encoding="utf-8") as f:
            f.write(content)
        module.__dict__["__file__"] = temp_file.name

        # Executing the modified content in the created module's namespace using importlib
        # This is safer than using exec() directly
        spec = util.spec_from_file_location(module_name, temp_file.name)
        if spec is None:
            raise ImportError(f"Could not create spec for module {module_name}")

        module = util.module_from_spec(spec)
        sys.modules[module_name] = module

        try:
            spec.loader.exec_module(module)
        except Exception as e:
            log.error(f"Error executing module {module_name}: {e}")
            raise

        frontmatter = extract_frontmatter(content)
        log.info(f"Loaded module: {module.__name__}")

        # Create and return the object if the class 'Tools' is found in the module
        if hasattr(module, "Tools"):
            return module.Tools(), frontmatter
        else:
            raise Exception("No Tools class found in the module")
    except Exception as e:
        log.error(f"Error loading module: {tool_id}: {e}")
        del sys.modules[module_name]  # Clean up
        raise e
    finally:
        os.unlink(temp_file.name)


def load_function_module_by_id(function_id, content=None):
    if content is None:
        function = Functions.get_function_by_id(function_id)
        if not function:
            raise Exception(f"Function not found: {function_id}")
        content = function.content

        content = replace_imports(content)
        Functions.update_function_by_id(function_id, {"content": content})
    else:
        frontmatter = extract_frontmatter(content)
        install_frontmatter_requirements(frontmatter.get("requirements", ""))

    module_name = f"function_{function_id}"
    module = types.ModuleType(module_name)
    sys.modules[module_name] = module

    # Create a temporary file and use it to define `__file__` so
    # that it works as expected from the module's perspective.
    temp_file = tempfile.NamedTemporaryFile(delete=False)
    temp_file.close()
    try:
        with open(temp_file.name, "w", encoding="utf-8") as f:
            f.write(content)
        module.__dict__["__file__"] = temp_file.name

        # Execute the modified content in the created module's namespace using importlib
        # This is safer than using exec() directly
        spec = util.spec_from_file_location(module_name, temp_file.name)
        if spec is None:
            raise ImportError(f"Could not create spec for module {module_name}")

        module = util.module_from_spec(spec)
        sys.modules[module_name] = module

        try:
            spec.loader.exec_module(module)
        except Exception as e:
            log.error(f"Error executing module {module_name}: {e}")
            raise

        frontmatter = extract_frontmatter(content)
        log.info(f"Loaded module: {module.__name__}")

        # Create appropriate object based on available class type in the module
        if hasattr(module, "Pipe"):
            return module.Pipe(), "pipe", frontmatter
        elif hasattr(module, "Filter"):
            return module.Filter(), "filter", frontmatter
        elif hasattr(module, "Action"):
            return module.Action(), "action", frontmatter
        else:
            raise Exception("No Function class found in the module")
    except Exception as e:
        log.error(f"Error loading module: {function_id}: {e}")
        del sys.modules[module_name]  # Cleanup by removing the module in case of error

        Functions.update_function_by_id(function_id, {"is_active": False})
        raise e
    finally:
        os.unlink(temp_file.name)


def install_frontmatter_requirements(requirements: str):
    if requirements:
        try:
            # Validate requirements to prevent command injection
            req_list = []
            for req in requirements.split(","):
                req = req.strip()
                # Validate package name format (alphanumeric, dash, underscore, dot)
                if re.match(r'^[A-Za-z0-9_.-]+[A-Za-z0-9_.-=<>]*$', req):
                    req_list.append(req)
                else:
                    log.warning(f"Skipping invalid requirement: {req}")

            if not req_list:
                log.warning("No valid requirements found after validation")
                return

            log.info(f"Installing requirements: {' '.join(req_list)}")

            # Use pip as a module with validated requirements
            subprocess.check_call(
                [sys.executable, "-m", "pip", "install"]
                + PIP_OPTIONS
                + req_list
                + PIP_PACKAGE_INDEX_OPTIONS,
                # Prevent shell injection
                shell=False
            )
        except Exception as e:
            log.error(f"Error installing packages: {' '.join(req_list)}")
            raise e

    else:
        log.info("No requirements found in frontmatter.")

