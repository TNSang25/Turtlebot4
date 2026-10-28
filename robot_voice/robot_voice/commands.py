# SPDX-License-Identifier: Apache-2.0
"""Strict Vietnamese navigation commands; unknown names never become goals."""

from dataclasses import dataclass
import re
import unicodedata


def normalize(text):
    text = unicodedata.normalize('NFKD', text.casefold().replace('đ', 'd'))
    text = ''.join(c for c in text if not unicodedata.combining(c))
    return ' '.join(re.sub(r'[^a-z0-9 ]+', ' ', text).split())


@dataclass(frozen=True)
class Command:
    kind: str
    destination: str = ''
    source: str = ''


class UnknownRegion(ValueError):
    pass


class CommandParser:
    def __init__(self, regions):
        self.aliases = {}
        self.names = {r.id: r.name for r in regions}
        for region in regions:
            aliases = [region.id, region.name, f'khu vực {region.id}', f'vùng {region.id}', *region.aliases]
            for alias in aliases:
                key = normalize(alias)
                if not key:
                    raise ValueError('Region aliases must not be empty')
                if key in self.aliases and self.aliases[key] != region.id:
                    raise ValueError(f'Ambiguous region alias: {alias}')
                self.aliases[key] = region.id

    def resolve(self, name):
        key = normalize(name)
        if key not in self.aliases:
            raise UnknownRegion(f'Không có khu vực "{name}". Các vùng: '+', '.join(f'{k}: {v}' for k, v in self.names.items()))
        return self.aliases[key]

    def parse(self, text):
        text = normalize(text)
        text = re.sub(r'^(?:robot )?(?:(?:hay|vui long|xin) )?', '', text)
        if text in ('dung', 'dung lai', 'huy', 'huy lenh', 'huy dieu huong', 'dung robot'):
            return Command('stop')
        # Only full utterances beginning with navigation intent are accepted.
        route = re.fullmatch(r'(?:di chuyen|di) tu (.+?) (?:den|toi) (.+?)(?: nhe)?', text)
        if route:
            return Command('navigate', self.resolve(route[2]), self.resolve(route[1]))
        destination = re.fullmatch(r'(?:di chuyen|di) (?:den|toi) (.+?)(?: nhe)?', text)
        if destination:
            return Command('navigate', self.resolve(destination[1]))
        raise ValueError('Lệnh chưa hợp lệ. Ví dụ: Đi đến khu vực A; Di chuyển từ khu vực A đến khu vực B; Dừng lại.')


def make_route(command, current_region=None):
    if command.kind != 'navigate':
        return []
    route = []
    if command.source and current_region != command.source and command.source != command.destination:
        route.append(command.source)
    route.append(command.destination)
    return route
