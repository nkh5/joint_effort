# CAD Git Workflow

## Why We Need This

SolidWorks files such as `.SLDPRT`, `.SLDASM`, and `.SLDDRW` are binary files.

Git cannot merge SolidWorks files like it can merge normal code.

Example:

Nick and Jack both have the same version of `arm.SLDPRT`.

Nick changes it and pushes the new version.

Jack still has the old version. If Jack later commits his copy, he could overwrite Nick's newer work.

To prevent this, we use **Git LFS File Locking**.

The rule is simple:

> Before editing a SolidWorks file, lock it.  
> When you are finished and your changes are pushed, unlock it.

---

# 1. One Time Setup

## Install Git LFS

### Mac

```bash
brew install git-lfs
git lfs install
```

### Windows

Install Git LFS, then run:

```bash
git lfs install
```

Each team member only needs to do this once.

---

# 3. Before Starting CAD Work

Always get the newest version first:

```bash
git pull
```

Then check which files are currently locked:

```bash
git lfs locks
```

If the file you want is available, lock it.

Example:

```bash
git lfs lock CAD/RightArm/Base.SLDPRT
```

You should see confirmation that the file is locked by you.

Now you can edit the file.

---

# 4. If Someone Else Has the File Locked

Do not edit it.

For example:

```text
CAD/RightArm/Base.SLDPRT
Locked by: Nick
```

Wait until Nick finishes and unlocks it.

You can still work on another CAD file at the same time.

For example:

```text
Nick
Base.SLDPRT

Jack
Wrist.SLDPRT

Pepper
SolderHolder.SLDPRT
```

This allows multiple people to work on CAD without editing the same file.

---

# 5. When You Finish Your CAD Work

First check what changed:

```bash
git status
```

Only add files you intentionally changed.

For example:

```bash
git add CAD/RightArm/Base.SLDPRT
```

Then commit:

```bash
git commit -m "Update right arm base geometry"
```

Push:

```bash
git push
```

After your work is safely pushed or merged, unlock the file:

```bash
git lfs unlock CAD/RightArm/Base.SLDPRT
```

Now another team member can work on it.

---

# 6. Very Important Rule

Do not blindly run:

```bash
git add .
```

for CAD work.

SolidWorks can modify files simply because they were opened or because references were updated.

Instead:

```bash
git status
```

Then add only the files you intentionally changed.

Example:

```bash
git add CAD/LeftArm/Shoulder.SLDPRT
git add CAD/LeftArm/LeftArm.SLDASM
```

---

# 7. Normal Daily Workflow

Every time you start working:

```bash
git pull
git lfs locks
git lfs lock CAD/path/to/file.SLDPRT
```

Do your SolidWorks work.

Then:

```bash
git status
git add CAD/path/to/file.SLDPRT
git commit -m "Describe CAD change"
git push
git lfs unlock CAD/path/to/file.SLDPRT
```

---

# 8. Example

Suppose we have:

```text
CAD/
    LeftArm/
        Base.SLDPRT
        Shoulder.SLDPRT
        Wrist.SLDPRT
        LeftArm.SLDASM
```

Nick wants to modify the base.

```bash
git pull
git lfs lock CAD/LeftArm/Base.SLDPRT
```

Jack runs:

```bash
git lfs locks
```

and sees:

```text
CAD/LeftArm/Base.SLDPRT    Nick
```

Jack should not modify `Base.SLDPRT`.

He can instead lock:

```bash
git lfs lock CAD/LeftArm/Wrist.SLDPRT
```

Now Nick and Jack can work at the same time without overwriting each other's CAD.

---

# 9. Assemblies

SolidWorks assemblies reference other SolidWorks files.

If you plan to modify both an assembly and one of its parts, lock both.

Example:

```bash
git lfs lock CAD/LeftArm/LeftArm.SLDASM
git lfs lock CAD/LeftArm/Base.SLDPRT
```

Do not lock every file in the project unless you actually need to modify them.

---

# Quick Reference

Check locks:

```bash
git lfs locks
```

Lock:

```bash
git lfs lock CAD/file.SLDPRT
```

Unlock:

```bash
git lfs unlock CAD/file.SLDPRT
```

Get newest files:

```bash
git pull
```

See what changed:

```bash
git status
```

Commit:

```bash
git add CAD/file.SLDPRT
git commit -m "Update CAD file"
git push
```

## Golden Rule

**If you do not own the lock, do not modify the CAD file.**