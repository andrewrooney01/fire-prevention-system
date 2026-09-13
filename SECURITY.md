# Credential and privacy protections

CI scans reachable Git history with a pinned, checksum-verified Gitleaks release and redacted output. GitHub-hosted scanning happens after a push; it does not undo disclosure. Enable local protection before committing:

```sh
brew install gitleaks
git config core.hooksPath .githooks
```

Check `git config --get core.hooksPath` first if you already use hooks; integrate the staged scan into your existing hook instead of replacing it. Do not put private keys, access tokens, live environment files, personal exports or databases in Git. Ignore patterns do not remove files already tracked. Keep personal content in private repositories. Public contact details and third-party license attribution may be intentional.

For a real credential exposure, revoke/rotate it at its provider, update its legitimate consumers, and assess history, Actions logs, artifacts and caches. Deleting a line alone is insufficient. Do not paste the credential into a public issue. Dependency alerts are enabled where supported; an empty initial alert list is not a completed dependency audit.
