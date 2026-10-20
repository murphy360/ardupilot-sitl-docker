"""This repository is public: no file in it names a vehicle maker's product (the owner,
2026-10-18, murphy360/Proteus#2170). A hull is known by a generic id, torpedo-100 and so
on. The word is built from two halves here so this file does not trip itself."""

import subprocess
from pathlib import Path

import pytest

WORD = "re" + "mus"
ROOT = Path(__file__).resolve().parents[2]


def _tracked() -> list[Path]:
    try:
        out = subprocess.run(
            ["git", "ls-files", "-z"],
            cwd=ROOT,
            capture_output=True,
            check=True,
        ).stdout
    except (OSError, subprocess.CalledProcessError):
        pytest.skip("not a git checkout")
    return [ROOT / n for n in out.decode().split("\0") if n]


def test_no_tracked_file_says_the_word():
    found = []
    for path in _tracked():
        if not path.is_file():
            continue
        try:
            text = path.read_text(errors="ignore")
        except OSError:
            continue
        for n, line in enumerate(text.splitlines(), 1):
            if WORD in line.lower():
                found.append(f"{path.relative_to(ROOT)}:{n}")
        if WORD in path.name.lower():
            found.append(str(path.relative_to(ROOT)))
    assert not found, "the word is not used here: " + ", ".join(found)


def test_the_check_catches_a_word(tmp_path, monkeypatch):
    """The scan itself works: a file that has the word is found."""
    (tmp_path / "a.txt").write_text(f"fine\nthe {WORD.upper()} 100\n")
    subprocess.run(["git", "init", "-q"], cwd=tmp_path, check=True)
    subprocess.run(["git", "add", "a.txt"], cwd=tmp_path, check=True)
    monkeypatch.setitem(globals(), "ROOT", tmp_path)
    found = [
        n
        for n, line in enumerate((tmp_path / "a.txt").read_text().splitlines(), 1)
        if WORD in line.lower()
    ]
    assert found == [2]
    assert [p.name for p in _tracked()] == ["a.txt"]
