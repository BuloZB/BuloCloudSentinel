from dataclasses import dataclass
from typing import Optional

@dataclass(frozen=True)
class User:
    id: int
    username: str
    email: str
    hashed_password: str
    is_active: bool = True
    is_superuser: bool = False
