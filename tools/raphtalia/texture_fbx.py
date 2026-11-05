"""Patch only FBX material records, retaining compressed rig/mesh/clip payloads."""
import argparse
import hashlib
import json
import struct
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
WORK = ROOT / 'build/raphtalia_tools/texturing'
DOC = ROOT / 'docs/raphtalia/texturing'


class Element:
    def __init__(self, name, count, properties, children, sentinel=False):
        self.name, self.count, self.properties = name, count, properties
        self.children, self.sentinel = children, sentinel

    def child(self, name):
        return next((n for n in self.children if n.name == name), None)

    def props(self):
        result, types, i = [], [], 0
        for _ in range(self.count):
            kind = chr(self.properties[i]); i += 1; types.append(kind)
            if kind in 'SR':
                size = struct.unpack_from('<I', self.properties, i)[0]; i += 4
                value = self.properties[i:i+size]; i += size
            else:
                fmt = {'L': 'q', 'I': 'i', 'Y': 'h', 'D': 'd', 'F': 'f', 'B': '?', 'C': 'c'}[kind]
                value = struct.unpack_from('<'+fmt, self.properties, i)[0]
                i += struct.calcsize(fmt)
            result.append(value)
        return types, result

    def set_props(self, values):
        types, _ = self.props()
        assert len(values) == len(types)
        buf = bytearray()
        for kind, value in zip(types, values):
            buf.extend(kind.encode())
            if kind in 'SR':
                buf.extend(struct.pack('<I', len(value))); buf.extend(value)
            else:
                fmt = {'L': 'q', 'I': 'i', 'Y': 'h', 'D': 'd', 'F': 'f', 'B': '?', 'C': 'c'}[kind]
                buf.extend(struct.pack('<'+fmt, value))
        self.properties = bytes(buf)

    def digest(self):
        digest = hashlib.sha256()
        digest.update(self.name)
        digest.update(struct.pack('<Q?', self.count, self.sentinel))
        digest.update(self.properties)
        for child in self.children:
            digest.update(child.digest())
        return digest.digest()

    def size(self, header):
        self.byte_size = header + len(self.name) + len(self.properties) + sum(c.size(header) for c in self.children)
        if self.sentinel:
            self.byte_size += header
        return self.byte_size

    def write(self, stream, wide):
        end = stream.tell() + self.byte_size
        stream.write(struct.pack('<QQQB' if wide else '<IIIB', end, self.count, len(self.properties), len(self.name)))
        stream.write(self.name); stream.write(self.properties)
        for child in self.children:
            child.write(stream, wide)
        if self.sentinel:
            stream.write(bytes(25 if wide else 13))
        assert stream.tell() == end


def read(path):
    data = Path(path).read_bytes()
    assert data[:23] == b'Kaydara FBX Binary  \0\x1a\0'
    version = struct.unpack_from('<I', data, 23)[0]
    wide = version >= 7500
    fmt, header = ('<QQQB', 25) if wide else ('<IIIB', 13)

    def node(offset):
        end, count, length, name_length = struct.unpack_from(fmt, data, offset)
        if not end:
            return None, offset+header
        p = offset+header
        name = data[p:p+name_length]; p += name_length
        properties = data[p:p+length]; p += length
        children, sentinel = [], False
        while p < end:
            child, p = node(p)
            if child is None:
                sentinel = True
                break
            children.append(child)
        assert p == end
        return Element(name, count, properties, children, sentinel), p
    roots, offset = [], 27
    while offset+header <= len(data):
        current, offset = node(offset)
        if current is None:
            break
        roots.append(current)
    return version, roots


def write(path, version, roots):
    wide = version >= 7500
    header = 25 if wide else 13
    for node in roots:
        node.size(header)
    with Path(path).open('wb') as stream:
        stream.write(b'Kaydara FBX Binary  \0\x1a\0' + struct.pack('<I', version))
        for node in roots:
            node.write(stream, wide)
        stream.write(bytes(header))
        stream.write(bytes.fromhex('fabcab09d0c8d466b176fb831cf7267e'))
        stream.write(bytes(4))
        stream.write(bytes(16 - stream.tell() % 16))
        stream.write(struct.pack('<I', version))
        stream.write(bytes(120))
        stream.write(bytes.fromhex('f85a8c6adef5d97eece90ce3758f290b'))


def patch(source, template, output):
    version, roots = read(source)
    _, stage = read(template)
    root = {n.name: n for n in roots}
    staged = {n.name: n for n in stage}
    objects = root[b'Objects'].children
    assert not any(n.name in (b'Texture', b'Video') for n in objects), (
        'Source already contains texture records. Use --source with the untextured FBX from the pre_texturing backup.')
    by_id = {n.props()[1][0]: n for n in objects}
    protected = {uid: n.digest() for uid, n in by_id.items() if n.name != b'Material'}
    materials = {n.props()[1][1].split(b'\0', 1)[0]: n for n in objects if n.name == b'Material'}
    mapping, appended, replacements = {}, [], {}
    new_id = max(by_id) + 1
    for node in staged[b'Objects'].children:
        if node.name not in (b'Material', b'Texture', b'Video'):
            continue
        _, props = node.props()
        old_id = props[0]
        if node.name == b'Material':
            target = materials[props[1].split(b'\0', 1)[0]]
            mapping[old_id] = target.props()[1][0]
            props[0] = mapping[old_id]
            node.set_props(props)
            replacements[props[0]] = node
        else:
            mapping[old_id] = new_id; new_id += 1
            props[0] = mapping[old_id]; node.set_props(props)
            # References are portable relative to assets/models, never the build tree.
            for key in (b'FileName', b'Filename', b'RelativeFilename'):
                leaf = node.child(key)
                if leaf:
                    name = leaf.props()[1][0].replace(b'\\', b'/').rsplit(b'/', 1)[-1]
                    leaf.set_props([b'../textures/raphtalia/'+name])
            prop_block = node.child(b'Properties70')
            if prop_block:
                for leaf in prop_block.children:
                    _, values = leaf.props()
                    if values and values[0] == b'Path':
                        name = values[-1].replace(b'\\', b'/').rsplit(b'/', 1)[-1]
                        values[-1] = b'../textures/raphtalia/'+name
                        leaf.set_props(values)
            appended.append(node)
    root[b'Objects'].children = [replacements.get(n.props()[1][0], n) for n in objects] + appended
    added_connections = []
    for node in staged[b'Connections'].children:
        _, values = node.props()
        if values[1] in mapping and values[2] in mapping:
            values[1] = mapping[values[1]]; values[2] = mapping[values[2]]
            node.set_props(values); added_connections.append(node)
    root[b'Connections'].children.extend(added_connections)
    definitions = root[b'Definitions']
    for key in (b'Texture', b'Video'):
        template_def = next(n for n in staged[b'Definitions'].children if n.name == b'ObjectType' and n.props()[1] == [key])
        count = sum(n.name == key for n in appended)
        template_def.child(b'Count').set_props([count])
        definitions.children.append(template_def)
    count_leaf = definitions.child(b'Count')
    count_leaf.set_props([count_leaf.props()[1][0] + len(appended)])
    write(output, version, roots)
    _, verified = read(output)
    verify = {n.name: n for n in verified}
    result_objects = {n.props()[1][0]: n for n in verify[b'Objects'].children}
    changed = [uid for uid, digest in protected.items() if result_objects[uid].digest() != digest]
    assert not changed, 'Non-material FBX payload changed'
    # Every pre-existing connection, including animation and skin links, is exact.
    old_links = root[b'Connections'].children[:-len(added_connections)]
    assert [n.digest() for n in verify[b'Connections'].children[:len(old_links)]] == [n.digest() for n in old_links]
    report = {'source': str(source), 'output': str(output), 'version': version,
              'preserved_non_material_objects': len(protected), 'changed_non_material_objects': changed,
              'original_connections_preserved': len(old_links),
              'materials': len(replacements), 'textures': sum(n.name == b'Texture' for n in appended),
              'videos': sum(n.name == b'Video' for n in appended),
              'new_connections': len(added_connections), 'sha256': hashlib.sha256(Path(output).read_bytes()).hexdigest()}
    (DOC / 'fbx_preservation.json').write_text(json.dumps(report, indent=2), encoding='utf-8')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    p = argparse.ArgumentParser()
    p.add_argument('--source', default='assets/models/Raphtalia_prepared.fbx')
    p.add_argument('--output', default='build/raphtalia_tools/texturing/Raphtalia_textured.fbx')
    args = p.parse_args()
    patch(ROOT / args.source, WORK / 'material_template.fbx', ROOT / args.output)
