# Repository contribution rules

- Use `lpc-robotics <lpc-robotics@users.noreply.github.com>` as both commit author
  and committer for every commit pushed to this repository.
- Do not add automated assistant identities or attribution trailers to commits
  or package maintainer metadata.
- Audit author and committer fields of all reachable commits before pushing
  imported history. Never push the independent local development history
  without auditing it first.
- Keep generated build/install directories, caches, runtime logs, binary
  releases, notebook checkpoints, and credentials out of version control.
- Preserve historical evidence and its hashes; new source changes require
  new release validation before claiming GPU acceptance.
