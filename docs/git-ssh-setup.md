# Git SSH Setup

Use SSH for this repository so each lab user or lab machine has its own GitHub identity and does not repeat browser/device authentication.

## Per-User Setup

1. Create a key outside the repository:

   ```bash
   ssh-keygen -t ed25519 -C "<user-or-machine> SDLsetup" -f ~/.ssh/id_ed25519_sdlsetup
   ```

2. Add only the public key to GitHub:

   ```bash
   cat ~/.ssh/id_ed25519_sdlsetup.pub
   ```

3. Use the SSH remote:

   ```bash
   git remote set-url origin git@github.com:erstool37/SDLsetup.git
   ```

4. Test access:

   ```bash
   ssh -T git@github.com
   ```

## Rules

- Never commit private keys.
- Never commit personal access tokens, passwords, `.env`, or GitHub credential files.
- Use one key per user or per shared lab machine so access can be revoked cleanly.
- Public keys are safe to share with GitHub; private keys stay on the machine that owns them.

