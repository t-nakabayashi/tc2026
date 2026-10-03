"""Bound configured NTP polls to 128 s, keeping the 256 s freshness gate."""
import argparse
import glob
from pathlib import Path
import re
import shlex
import shutil
import subprocess


def bound_polls(text):
    lines = []
    for line in text.splitlines(keepends=True):
        body, marker, comment = line.partition('#')
        fields = body.split()
        if fields and fields[0].lower() in ('server', 'pool', 'peer'):
            for option in ('minpoll', 'maxpoll'):
                match = re.search(r'\b' + option + r'\s+(-?\d+)\b', body)
                if match and int(match[1]) > 7:
                    body = body[:match.start(1)] + '7' + body[match.end(1):]
            if 'maxpoll' not in fields:
                body = body.rstrip() + ' maxpoll 7' + (' ' if marker else '\n')
            line = body + marker + comment
        lines.append(line)
    return ''.join(lines)


def configuration_files(config):
    """Follow existing include/confdir/sourcedir files without replacing sources."""
    found = set()
    pending = [config]
    while pending:
        path = pending.pop().resolve()
        if path in found:
            continue
        found.add(path)
        for line in path.read_text().splitlines():
            fields = shlex.split(line, comments=True)
            if not fields or fields[0] not in ('include', 'confdir', 'sourcedir'):
                continue
            for entry in fields[1:]:
                if not Path(entry).is_absolute():
                    raise ValueError(f'Absolute chrony include path required: {entry}')
                pattern = entry
                if fields[0] != 'include':
                    pattern += '/*.conf' if fields[0] == 'confdir' else '/*.sources'
                pending.extend(Path(p) for p in glob.glob(pattern))
    return sorted(found)


def apply(config, backup):
    changes = {}
    for path in configuration_files(config):
        original = path.read_text()
        updated = bound_polls(original)
        if original != updated:
            changes[path] = (original, updated)
    for path in changes:
        saved = backup / path.relative_to('/')
        saved.parent.mkdir(parents=True, exist_ok=True)
        if saved.exists():
            raise FileExistsError(saved)
        shutil.copy2(path, saved)
    try:
        for path, (_, updated) in changes.items():
            path.write_text(updated)
        subprocess.run(['chronyd', '-p', '-f', str(config)],
                       check=True, stdout=subprocess.DEVNULL, timeout=10)
    except BaseException:
        for path, (original, _) in changes.items():
            path.write_text(original)
        raise
    for path in changes:
        print(f'NTP maxpoll <= 7: {path}')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, default=Path('/etc/chrony/chrony.conf'))
    parser.add_argument('--backup-directory', type=Path, required=True)
    args = parser.parse_args()
    apply(args.config, args.backup_directory)
