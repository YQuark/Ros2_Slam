# Errors

Command failures and integration errors.

---

## [ERR-20260412-001] git index mode normalization failed on mounted workspace

**Logged**: 2026-04-12T00:00:00+08:00
**Priority**: low
**Status**: pending
**Area**: tooling

### Summary
`git update-index --chmod=-x` failed on the SMB-mounted ROS workspace with a read-only temporary object error.

### Details
The mounted workspace can allow source edits while rejecting Git object database writes for some index operations. For file mode noise, prefer `git config --local core.fileMode false` when permitted instead of forcing index chmod updates.

### Metadata
- Source: error
- Related Files: launch_scripts/robot.sh
- Tags: git, smb, filemode
