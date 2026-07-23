from robot_navigation_guard.guard import GoalGuard


def test_old_goal_never_reopens_after_recovery() -> None:
    guard = GoalGuard(safe=True)
    assert guard.observe_goal("old")
    assert guard.revoke()
    guard.recover()
    assert not guard.observe_goal("old")
    assert guard.observe_goal("new")
