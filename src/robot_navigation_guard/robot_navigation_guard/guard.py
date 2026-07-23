from dataclasses import dataclass


@dataclass
class GoalGuard:
    safe: bool = False
    active_uuid: str = ""
    revoked_uuid: str = ""
    generation: int = 0

    def observe_goal(self, uuid: str) -> bool:
        self.active_uuid = uuid
        return self.safe and bool(uuid) and uuid != self.revoked_uuid

    def revoke(self) -> bool:
        changed = self.safe or bool(self.active_uuid)
        self.safe = False
        if self.active_uuid:
            self.revoked_uuid = self.active_uuid
        self.active_uuid = ""
        if changed:
            self.generation += 1
        return changed

    def recover(self) -> None:
        self.safe = True
