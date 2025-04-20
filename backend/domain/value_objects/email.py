from dataclasses import dataclass
import re

@dataclass(frozen=True)
class Email:
    address: str

    def __post_init__(self):
        if not self._is_valid_email(self.address):
            raise ValueError(f"Invalid email address: {self.address}")

    @staticmethod
    def _is_valid_email(email: str) -> bool:
        pattern = r"^[\\w\\.-]+@[\\w\\.-]+\\.\\w+$"
        return re.match(pattern, email) is not None
