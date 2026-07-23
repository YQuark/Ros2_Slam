from dataclasses import dataclass


@dataclass
class GoalGuard:
    safe: bool = False
    active_uuid: str = ""
    revoked_uuid: str = ""
    revoke_generation: int = 0
    goal_generation: int = 0
    cancel_pending: bool = False
    cancel_attempts: int = 0

    def observe_goal(self, uuid: str) -> bool:
        if not self.safe or not uuid or uuid == self.revoked_uuid or self.active_uuid:
            return False
        self.active_uuid = uuid
        self.goal_generation += 1
        self.cancel_pending = False
        self.cancel_attempts = 0
        return True

    def revoke(self) -> bool:
        changed = self.safe or (bool(self.active_uuid) and not self.cancel_pending)
        self.safe = False
        if self.active_uuid:
            self.revoked_uuid = self.active_uuid
            self.cancel_pending = True
        if changed:
            self.revoke_generation += 1
        return changed

    def recover(self) -> None:
        self.safe = True

    def note_cancel_attempt(self) -> None:
        if self.cancel_pending:
            self.cancel_attempts += 1

    def terminate(self, uuid: str) -> None:
        if uuid == self.active_uuid:
            self.active_uuid = ""
            self.cancel_pending = False

    @property
    def ready(self) -> bool:
        return self.safe and not self.active_uuid and not self.cancel_pending
