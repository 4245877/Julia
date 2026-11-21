"""Check local Markdown links in project documentation (generated trees excluded)."""
import json
from pathlib import Path
import re
import shutil
import subprocess
from urllib.parse import unquote, urlsplit

ROOT = Path(__file__).resolve().parents[1]


def main():
    command = (['rg', '--files', '--glob', '*.md'] if shutil.which('rg') else
               ['git', '-c', f'core.excludesFile={ROOT / ".gitignore"}', 'ls-files',
                '--cached', '--others', '--exclude-standard', '--', '*.md'])
    listing = subprocess.check_output(command, cwd=ROOT, text=True, encoding='utf-8')
    documents = sorted(ROOT/path for path in listing.splitlines())
    missing = []
    count = 0
    for document in documents:
        text = document.read_text(encoding='utf-8')
        text = re.sub(r'```.*?```', '', text, flags=re.DOTALL)
        for target in re.findall(r'\[[^\]]*\]\(([^)]+)\)', text):
            target = target.strip().strip('<>')
            parts = urlsplit(target)
            if parts.scheme or parts.netloc:
                continue
            count += 1
            path = unquote(parts.path)
            destination = (ROOT/path.lstrip('/')) if path.startswith('/') else (document.parent/path)
            if path and not destination.exists():
                missing.append(dict(document=document.relative_to(ROOT).as_posix(), target=target))
    print(json.dumps(dict(documents=len(documents), local_links=count, missing=missing), ensure_ascii=False, indent=2))
    return bool(missing)


if __name__ == '__main__':
    raise SystemExit(main())
