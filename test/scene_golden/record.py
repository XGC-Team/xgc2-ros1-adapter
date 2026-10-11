#!/usr/bin/env python3.8
"""Record the pinned scene runtime's public behavior without ROS or TCP."""

import argparse
import contextlib
import copy
import hashlib
import io
import json
import math
import os
from pathlib import Path
import re
import sys
import tempfile
import threading
import time
from unittest import mock

sys.dont_write_bytecode = True
import yaml

HERE = Path(__file__).resolve().parent


def case(filename, name):
    return filename + '.py:test_' + name


def store_case(name):
    return case('test_scene_store', name)


def box(identity='box'):
    return {'id': identity, 'pose': {'position': [1, 2, 3]}, 'parts': [
        {'id': 'body', 'geometry': {'type': 'box', 'size': [1, 2, 3]}}]}


def scene(*obstacles):
    return {'schema': 'xgc2.scene.v1', 'id': 'test', 'frame': 'world',
            'obstacles': list(obstacles)}


def tetrahedron():
    return {'type': 'convex', 'vertices': [[0, 0, 0], [1, 0, 0], [0, 1, 0], [0, 0, 1]],
            'triangles': [0, 1, 2, 0, 3, 1, 0, 2, 3, 1, 3, 2]}


def varied_scene(count):
    # The test's non-English display name is replaced by an English name.
    items = []
    for index in range(count):
        item = box('o{:03d}'.format(index))
        item['name'] = ('Obstacle {} with a long descriptive name that PyYAML '
                        'may fold at eighty columns').format(index)
        item['parts'].append({'id': 'hull', 'pose': {'position': [0, 0, 1e-7 * index]},
                              'geometry': tetrahedron()})
        if index % 3 == 0:
            item['motion'] = {'type': 'circle', 'center': [1, 1.5, 3], 'radius': 0.5,
                              'angular_speed': 0.1, 'phase': math.pi / 2}
        items.append(item)
    return scene(*items)


class Scenario:
    """Capture inputs before calls and observe files and native hooks afterwards."""

    def __init__(self, category, name, sources):
        self.name = category + '/' + name
        self.value = {'scenario': self.name, 'derived_from': sources,
                      'setup': {'files': {}, 'directories': [], 'stores': {}}, 'steps': []}
        self.temporary = tempfile.TemporaryDirectory(prefix='scene-golden-')
        self.root = Path(self.temporary.name)
        self.tokens = {str(self.root): '$work', str(HERE / 'fixtures'): '$fixtures'}
        self.paths = set()
        self.stores = {}
        self.options = {}
        self.now = 100.0
        self.epochs = 0
        self.serial = 0
        self.apply_outcome = {'outcome': 'ok'}
        self.apply_calls = []
        self.motion_calls = []

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.temporary.cleanup()

    def token(self, actual, token):
        self.tokens[str(actual)] = token

    def normalize(self, value):
        if isinstance(value, str):
            for actual, token in sorted(self.tokens.items(), key=lambda item: -len(item[0])):
                value = value.replace(actual, token)
            return value
        if isinstance(value, dict):
            return {self.normalize(key): self.normalize(item) for key, item in value.items()}
        if isinstance(value, (list, tuple)):
            return [self.normalize(item) for item in value]
        return value

    def path(self, relative):
        path = self.root / relative
        self.paths.add(path)
        return path

    def files(self):
        paths = self.paths | {p for p in self.root.rglob('*') if p.is_file() or p.is_symlink()}
        result = {}
        for path in sorted(paths):
            if path.is_symlink():
                result[str(path)] = {'symlink': os.readlink(str(path))}
            elif path.is_file():
                result[str(path)] = {'contents': path.read_text(encoding='utf-8'),
                                     'mode': path.stat().st_mode & 0o777}
            else:
                result[str(path)] = {'contents': None}
        return result

    def initial_file(self, relative, contents, mode=0o644):
        path = self.path(relative)
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(contents, encoding='utf-8')
        path.chmod(mode)
        self.value['setup']['files'][str(path)] = {'contents': contents, 'mode': mode}
        return path

    def directory(self, relative):
        path = self.root / relative
        path.mkdir(parents=True, exist_ok=True)
        self.value['setup']['directories'].append(str(path))
        return path

    def fixture(self, name):
        path = HERE / 'fixtures' / name
        self.value['setup'].setdefault('executables', {})[str(path)] = {
            'contents': path.read_text(encoding='utf-8'), 'mode': 0o755}
        return str(path)

    def call(self, operation, request, function, effects=False, **extra):
        step = {'operation': operation, 'request': copy.deepcopy(request)}
        step.update(extra)
        try:
            step['response'] = {'value': copy.deepcopy(function())}
        except (ValueError, OSError, TypeError, KeyError, yaml.YAMLError) as error:
            step['response'] = {'error': {'type': type(error).__name__, 'message': str(error)}}
        if effects:
            step['effects'] = self.effects()
        self.value['steps'].append(step)
        return step['response']

    def effects(self):
        result = {'files': self.files(), 'apply': self.apply_calls, 'motion': self.motion_calls}
        self.apply_calls = []
        self.motion_calls = []
        return result

    def create_store(self, initial, label='main', **options):
        config = dict(options, document=copy.deepcopy(initial), clock=100.0)
        self.value['setup']['stores'][label] = config
        self.options[label] = dict(options)
        self.options[label]['initial'] = copy.deepcopy(initial)
        self._open_store(label)
        self.value['setup']['stores'][label]['initial_response'] = self.stores[label].envelope()
        self.value['setup']['stores'][label]['initial_effects'] = self.effects()
        return self.stores[label]

    def _open_store(self, label):
        options = dict(self.options[label])
        initial = options.pop('initial')
        trace = options.pop('apply_hook', False)
        motion = options.pop('motion_hook', False)
        if options.get('source'):
            initial, _ = store_module.load(options['source'])
        def apply(document, epoch, revision):
            outcome = dict(self.apply_outcome)
            self.apply_calls.append({'request': {'document': copy.deepcopy(document),
                                                 'epoch': epoch, 'revision': revision},
                                     'response': outcome, 'files': self.files()})
            if outcome['outcome'] != 'ok':
                raise document_module.SceneError(outcome['message'])
        def motion_hook(operation, epoch, revision):
            self.motion_calls.append({'request': {'operation': operation, 'epoch': epoch,
                                                  'revision': revision}, 'response': {'outcome': 'ok'}})
        store = store_module.SceneStore(initial, clock=lambda: self.now,
                                       apply=apply if trace else None,
                                       motion=motion_hook if motion else None, **options)
        token = '$epoch' if self.epochs == 0 else '$epoch{}'.format(self.epochs)
        self.epochs += 1
        self.token(store.epoch, token)
        self.stores[label] = store

    def command(self, operation=None, label='main', request=None, disk_error=None, **values):
        store = self.stores[label]
        if request is None:
            request = dict(requestId='r{}'.format(self.serial), expectedEpoch=store.epoch,
                           expectedRevision=store.revision, operation=operation, **values)
            self.serial += 1
        def execute():
            if disk_error:
                with mock.patch.object(store_module.os, 'replace', side_effect=OSError(disk_error)):
                    return store.command(request)
            return store.command(request)
        extra = {'store': label}
        if disk_error:
            extra['fault'] = {'operation': 'file.replace', 'message': disk_error}
        return self.call('store.command', request, execute, effects=True, **extra)

    def envelope(self, label='main'):
        return self.call('store.envelope', {}, self.stores[label].envelope, store=label)

    def states(self, label='main'):
        return self.call('store.states', {}, self.stores[label].states, store=label)

    def restart(self, label='main'):
        def reopen():
            self._open_store(label)
            return self.stores[label].envelope()
        return self.call('store.restart', self.options[label], reopen, effects=True, store=label)

    def mutate(self, path, contents=None, symlink=None):
        def change():
            if symlink is not None:
                path.unlink()
                path.symlink_to(symlink)
            else:
                path.write_text(contents, encoding='utf-8')
            return None
        request = {'path': str(path)}
        request.update({'symlink': str(symlink)} if symlink is not None else {'contents': contents})
        return self.call('file.mutate', request, change, effects=True)

    def advance(self, seconds):
        def advance_clock():
            self.now += seconds
            return self.now
        return self.call('clock.advance', {'seconds': seconds}, advance_clock)

    def simulator(self, outcome, message=None):
        def set_outcome():
            self.apply_outcome = {'outcome': outcome}
            if message:
                self.apply_outcome['message'] = message
            return None
        request = {'outcome': outcome}
        if message:
            request['message'] = message
        return self.call('simulator.apply_outcome', request, set_outcome)

    def finish(self):
        return self.normalize(self.value)


def document_scenarios():
    values = []
    with Scenario('document', 'visual-color', [store_case('obstacle_visual_color_is_product_amber')]) as s:
        gray = box()
        gray['parts'][0]['color'] = [0.5, 0.5, 0.5, 1]
        for value in (scene(gray), scene(box())):
            s.call('document.validate', value, lambda value=value: document_module.document(value))
        values.append(s.finish())
    with Scenario('document', 'compound-sizes', [store_case('compounds_preserve_parts_and_geometry_has_explicit_sizes')]) as s:
        arch = box('arch')
        arch['parts'] = [dict(id='left', geometry={'type': 'box', 'size': [0.2, 1, 2]}),
                         dict(id='right', pose={'position': [2, 0, 0]}, geometry={'type': 'sphere', 'radius': 0.2}),
                         dict(id='top', pose={'position': [1, 0, 2]}, geometry={'type': 'capsule', 'radius': 0.2, 'height': 2})]
        value = scene(arch)
        s.call('document.validate', value, lambda: document_module.document(value))
        values.append(s.finish())
    with Scenario('document', 'invalid-geometry', [store_case('invalid_geometry_does_not_become_another_shape')]) as s:
        for value in ({'type': 'cone', 'radius': 2}, {'type': 'sphere', 'radius': -1},
                      {'type': 'cylinder', 'radius': 1, 'height': 0},
                      {'type': 'box', 'size': [1, float('nan'), 1]},
                      {'type': 'box', 'size': [True, 1, 1]}):
            s.call('document.geometry', {'json': json.dumps(value)},
                   lambda value=value: document_module.geometry(value))
        values.append(s.finish())
    with Scenario('document', 'duplicate-and-huge', [store_case('duplicate_fields_and_huge_numbers_are_rejected')]) as s:
        text = '{"operation":"get","operation":"clear"}'
        s.call('document.parse_json', {'json': text},
               lambda: json.loads(text, object_pairs_hook=store_module.unique_object))
        huge = {'type': 'sphere', 'radius': 10**1000}
        s.call('document.geometry', {'json': json.dumps(huge)}, lambda: document_module.geometry(huge))
        path = s.initial_file('scene.yaml', 'schema: xgc2.scene.v1\nid: first\nid: second\nframe: world\nobstacles: []\n')
        s.call('document.load', {'path': str(path)}, lambda: store_module.load(path), effects=True)
        values.append(s.finish())
    with Scenario('document', 'convex-manifold', [store_case('convex_manifold_required_without_filling_compound_holes')]) as s:
        tetra = tetrahedron()
        broken = copy.deepcopy(tetra)
        broken['triangles'][-3:] = broken['triangles'][:3]
        for value in (tetra, broken):
            s.call('document.geometry', {'json': json.dumps(value)}, lambda value=value: document_module.geometry(value))
        values.append(s.finish())
    with Scenario('document', 'motion-start', [store_case('motion_path_and_initial_pose_cannot_disagree')]) as s:
        moving = box()
        moving['motion'] = {'type': 'ping_pong', 'point_a': [1, 2, 3], 'point_b': [4, 2, 3], 'speed': 1}
        for position in ([1, 2, 3], [2, 2, 3]):
            moving['pose']['position'] = position
            s.call('document.validate', scene(moving), lambda: document_module.document(scene(moving)))
        moving['motion'] = {'type': 'circle', 'center': [1, 2, 3], 'radius': 1, 'angular_speed': 1, 'phase': 0}
        s.call('document.validate', scene(moving), lambda: document_module.document(scene(moving)))
        values.append(s.finish())
    with Scenario('document', 'identifiers', [store_case('ids_are_stable_through_delete_copy_undo_and_reload')]) as s:
        # Explicit grammar boundaries requested by the package brief.
        s.value['brief_boundaries'] = ['id grammar']
        for identity in ('a.b-c_1', '', '../escape', 'x' * 129):
            value = scene(box(identity))
            s.call('document.validate', value, lambda value=value: document_module.document(value))
        values.append(s.finish())
    with Scenario('document', 'limits', [store_case('scene_limits_still_apply_to_incremental_edits')]) as s:
        s.value['brief_boundaries'] = ['129 parts per obstacle']
        wide = box('wide')
        wide['parts'] = [{'id': 'p{}'.format(i), 'geometry': {'type': 'sphere', 'radius': 0.1}} for i in range(128)]
        oversized = copy.deepcopy(wide)
        oversized['parts'].append({'id': 'p128', 'geometry': {'type': 'sphere', 'radius': 0.1}})
        full = scene(*[dict(wide, id='w{}'.format(i)) for i in range(32)], box('one-more-part'))
        for value in (scene(*[box('o{}'.format(i)) for i in range(513)]), scene(oversized), full,
                      scene(box('a'), box('a'))):
            s.call('document.validate', value, lambda value=value: document_module.document(value))
        values.append(s.finish())
    with Scenario('document', 'empty-snapshot', [store_case('full_snapshot_can_be_empty_but_bad_document_cannot_clear_it')]) as s:
        for value in (scene(), {'obstacles': []}):
            s.call('document.validate', value, lambda value=value: document_module.document(value))
        values.append(s.finish())
    return values


def store_scenarios():
    values = []
    with Scenario('store', 'stable-ids-undo-redo', [store_case('ids_are_stable_through_delete_copy_undo_and_reload')]) as s:
        s.create_store(scene(box('a'), box('b'), box('c')))
        s.command('delete', id='b'); s.envelope()
        s.command('undo'); s.envelope()
        s.command('redo'); s.command('add', obstacle=box('copy')); s.command('redo')
        values.append(s.finish())
    with Scenario('store', 'identity-and-stale', [store_case('stale_epoch_revision_and_request_reuse_cannot_overwrite')]) as s:
        store = s.create_store(scene(box()))
        first = dict(requestId='same', expectedEpoch=store.epoch, expectedRevision=1,
                     operation='add', obstacle=box('new'))
        s.command(request=first); s.command(request=first)
        s.command(request=dict(first, operation='clear'))
        s.command(request=dict(first, requestId='stale'))
        s.command(request=dict(first, requestId='epoch', expectedRevision=2, expectedEpoch='old'))
        s.envelope()
        values.append(s.finish())
    with Scenario('store', 'simulator-rejected', [store_case('simulator_rejection_does_not_commit_or_create_undo_entry')]) as s:
        s.create_store(scene(box()), apply_hook=True)
        s.simulator('rejected', 'collision update rejected')
        s.command('clear'); s.envelope(); s.command('undo')
        values.append(s.finish())
    with Scenario('store', 'autosave-restart', [store_case('edits_undo_and_redo_automatically_survive_restart')]) as s:
        path = s.initial_file('scene.yaml', yaml.safe_dump(scene(box())), 0o640)
        s.create_store(scene(box()), source=str(path))
        for operation, kwargs in (('add', {'obstacle': box('new')}), ('undo', {}), ('redo', {})):
            s.command(operation, **kwargs)
            s.options['reopened'] = dict(s.options['main'])
            s.restart('reopened')
        values.append(s.finish())
    with Scenario('store', 'external-edit-reload', [store_case('agent_file_edits_require_reload_and_are_never_overwritten'),
                                                  case('test_scene_frozen', 'unfrozen_default_keeps_every_existing_channel_open')]) as s:
        path = s.initial_file('scene.yaml', yaml.safe_dump(scene(box())))
        s.create_store(scene(box()), source=str(path))
        s.command('add', obstacle=box('new')); s.command('undo'); s.command('save')
        s.mutate(path, yaml.safe_dump(scene(box('agent'))))
        s.command('clear'); s.command('save'); s.command('reload')
        s.command('add', obstacle=box('human')); s.command('save', path='another.yaml')
        values.append(s.finish())
    with Scenario('store', 'save-failure-retry', [store_case('save_failure_keeps_live_edit_and_retry_does_not_apply_it_twice')]) as s:
        path = s.initial_file('scene.yaml', yaml.safe_dump(scene(box())))
        store = s.create_store(scene(box()), source=str(path), apply_hook=True)
        request = dict(requestId='edit', expectedEpoch=store.epoch, expectedRevision=1,
                       operation='add', obstacle=box('new'))
        s.command(request=request, disk_error='disk full'); s.envelope()
        s.command(request=request); s.command('save'); s.envelope()
        values.append(s.finish())
    with Scenario('store', 'lost-reply-resync', [store_case('lost_reply_and_partial_apply_recover_without_reusing_revision')]) as s:
        s.create_store(scene(box()), apply_hook=True)
        s.simulator('unknown', 'reply lost after simulator changed')
        s.command('clear'); s.envelope()
        s.simulator('ok'); s.command('resync'); s.envelope(); s.command('undo')
        values.append(s.finish())
    with Scenario('store', 'native-failure', [case('test_xrpc_control', 'failed_native_effect_does_not_promote_desired_or_persisted')]) as s:
        s.create_store(scene(), apply_hook=True)
        s.simulator('unknown', 'native outcome unknown')
        s.command('replace', document=dict(scene(), id='replacement')); s.envelope()
        values.append(s.finish())
    with Scenario('store', 'startup-digest', [store_case('source_changed_during_startup_is_not_overwritten')]) as s:
        path = s.initial_file('scene.yaml', yaml.safe_dump(scene(box())))
        loaded = s.call('document.load', {'path': str(path)}, lambda: store_module.load(path))
        initial, digest = loaded['value']
        s.mutate(path, '# external edit during startup\n' + path.read_text())
        s.options['main'] = {'initial': initial, 'source': str(path), 'source_digest': digest}
        s.restart()
        s.command('save')
        values.append(s.finish())
    with Scenario('store', 'motion-clock', [store_case('motion_clock_and_saved_initial_definition_remain_separate')]) as s:
        moving = box()
        moving['motion'] = {'type': 'constant_twist', 'linear': [2, 0, 0], 'angular': [0, 0, 0]}
        s.create_store(scene(moving), motion_hook=True)
        s.command('play'); s.advance(3); s.states(); s.command('pause')
        s.advance(10); s.states(); s.envelope(); s.command('reset'); s.states()
        values.append(s.finish())
    with Scenario('store', 'empty-snapshot', [store_case('full_snapshot_can_be_empty_but_bad_document_cannot_clear_it')]) as s:
        s.create_store(scene(box()))
        s.command('replace', document={'obstacles': []}); s.envelope()
        s.command('clear'); s.command(request={'operation': 'get'})
        values.append(s.finish())
    with Scenario('store', 'working-isolation', [store_case('experiment_working_files_are_isolated_and_persist_edits')]) as s:
        source = s.initial_file('shared.yaml', yaml.safe_dump(scene(box('shared'))))
        for label in ('a', 'b'):
            s.create_store(scene(box('shared')), label=label, source=str(source), save_root=str(s.root),
                           working_file=str(s.path('experiment-' + label + '/scene.yaml')))
        for label in ('a', 'b'):
            s.command('add', label=label, obstacle=box(label + '-only'))
        s.command('undo', label='a'); s.command('redo', label='a')
        s.command('save', label='a'); s.command('reload', label='a')
        s.restart('a'); s.restart('b')
        values.append(s.finish())
    for name, failure, source_case in (
            ('working-before-apply', False, 'working_file_is_created_on_first_edit_before_apply'),
            ('working-copy-failure', True, 'first_working_copy_write_failure_does_not_apply_or_change_source')):
        with Scenario('store', name, [store_case(source_case)]) as s:
            source = s.initial_file('shared.yaml', yaml.safe_dump(scene(box('shared'))))
            s.create_store(scene(box('shared')), source=str(source), save_root=str(s.root),
                           working_file=str(s.path('experiment/scene.yaml')), apply_hook=True)
            s.command('add', obstacle=box('experiment-only'), disk_error='disk full' if failure else None)
            s.envelope()
            values.append(s.finish())
    with Scenario('store', 'working-reopen', [store_case('existing_working_file_wins_over_changed_source_on_reopen')]) as s:
        source = s.initial_file('shared.yaml', yaml.safe_dump(scene(box('shared'))))
        s.create_store(scene(box('shared')), source=str(source), save_root=str(s.root),
                       working_file=str(s.path('experiment/scene.yaml')))
        s.command('add', obstacle=box('experiment-only'))
        s.mutate(source, yaml.safe_dump(scene(box('asset-changed-later'))))
        s.restart(); s.command('reload')
        values.append(s.finish())
    with Scenario('store', 'frozen-rejections', [case('test_scene_frozen', name) for name in (
            'every_edit_save_and_reload_channel_is_rejected_with_an_explicit_error',
            'rejection_is_a_result_not_a_silent_drop_or_exception')]) as s:
        store = s.create_store(scene(box()), frozen=True, apply_hook=True)
        first = dict(requestId='frozen-clear', expectedEpoch=store.epoch, expectedRevision=1, operation='clear')
        s.command(request=first); s.command(request=first)
        for operation, kwargs in (('add', {'obstacle': box('new')}), ('update', {'obstacle': box()}),
                                  ('delete', {'id': 'box'}), ('clear', {}),
                                  ('replace', {'document': scene(box('other'))}), ('undo', {}),
                                  ('redo', {}), ('save', {}), ('reload', {})):
            s.command(operation, **kwargs)
        values.append(s.finish())
    with Scenario('store', 'frozen-read-motion-resync', [case('test_scene_frozen', 'frozen_scene_still_serves_reads_motion_clock_and_resync')]) as s:
        s.create_store(scene(box()), frozen=True, apply_hook=True, motion_hook=True)
        s.command(request={'operation': 'get'})
        for operation in ('play', 'pause', 'reset', 'resync'):
            s.command(operation)
        s.envelope()
        values.append(s.finish())
    with Scenario('store', 'frozen-source', [case('test_scene_frozen', 'frozen_source_yaml_is_never_reloaded_or_written')]) as s:
        source = s.initial_file('scene.yaml', yaml.safe_dump(scene(box())))
        s.create_store(scene(box()), source=str(source), frozen=True)
        s.mutate(source, yaml.safe_dump(scene(box('agent'))))
        s.command('reload'); s.command('save'); s.envelope()
        values.append(s.finish())
    with Scenario('store', 'incremental-yaml', [store_case('incremental_yaml_equals_a_full_dump')]) as s:
        writer = store_module.DocumentYaml()
        for count in (0, 1, 7):
            value = document_module.document(varied_scene(count))
            s.call('document.dump_yaml', value, lambda value=value: {
                'incremental': writer.dump(value), 'full': store_module.dump_yaml(value)})
        values.append(s.finish())
    for name, initial, added in (
            ('obstacle-limit', scene(*[box('o{}'.format(i)) for i in range(512)]), box('one-too-many')),
            ('part-limit', scene(*[{'id': 'w{}'.format(i), 'pose': {'position': [1, 2, 3]},
                                   'parts': [{'id': 'p{}'.format(p), 'geometry': {'type': 'sphere', 'radius': 0.1}}
                                             for p in range(128)]} for i in range(32)]), box('one-more-part')),
            ('duplicate-obstacle', scene(box('a'), box('b')), box('a'))):
        with Scenario('store', name, [store_case('scene_limits_still_apply_to_incremental_edits')]) as s:
            s.create_store(initial)
            # The command already contains the exact unchanged envelope; avoid
            # duplicating it in setup for the two large limit fixtures.
            del s.value['setup']['stores']['main']['initial_response']
            s.command('add', obstacle=added)
            values.append(s.finish())
    return values


def motion_scenarios():
    values = []
    twist = box()
    twist['motion'] = {'type': 'constant_twist', 'linear': [2, 0, 0], 'angular': [0, 0, 0]}
    ping = box()
    ping['motion'] = {'type': 'ping_pong', 'point_a': [1, 2, 3], 'point_b': [4, 2, 3], 'speed': 1}
    circle = box()
    circle['pose']['position'] = [2, 2, 3]
    circle['motion'] = {'type': 'circle', 'center': [1, 2, 3], 'radius': 1, 'angular_speed': 1, 'phase': 0}
    pillar = {'id': 'pillar-0', 'name': 'Pillar "0"', 'pose': {'position': [0, -2, 1]},
              'parts': [{'id': 'body', 'geometry': {'type': 'cylinder', 'radius': 0.2, 'height': 4}},
                        {'id': 'cap', 'pose': {'position': [0, 0, 2.2]},
                         'geometry': {'type': 'sphere', 'radius': 0.25}},
                        {'id': 'base', 'geometry': {'type': 'box', 'size': [0.5, 0.5, 0.2]}}],
              'motion': {'type': 'constant_twist', 'linear': [0.1, 0, 0], 'angular': [0, 0, 0.3]}}
    for name, obstacle, sources, times in (
            ('hold', box(), [store_case('ids_are_stable_through_delete_copy_undo_and_reload')], (0, 2)),
            ('constant-twist', twist, [store_case('motion_clock_and_saved_initial_definition_remain_separate')], (0, 3)),
            ('ping-pong', ping, [store_case('motion_path_and_initial_pose_cannot_disagree')], (0, 2, 3, 4, 6)),
            ('circle', circle, [store_case('motion_path_and_initial_pose_cannot_disagree')], (0, 2)),
            ('circle-phase', varied_scene(1)['obstacles'][0], [store_case('incremental_yaml_equals_a_full_dump')], (0, 2)),
            ('rotating-twist', pillar, [case('test_scene_projection', 'snapshots_markers_and_states_equal_the_uncached_projections')], (0, 2))):
        with Scenario('motion', name, sources) as s:
            normalized = document_module.obstacle(obstacle)
            s.value['setup']['obstacle'] = obstacle
            s.call('document.validate', scene(obstacle), lambda: document_module.document(scene(obstacle)))
            for elapsed in times:
                for playing in (False, True):
                    request = {'obstacle': normalized, 'elapsed': float(elapsed), 'playing': playing}
                    s.call('motion.state', request,
                           lambda elapsed=elapsed, playing=playing: motion_module.state(normalized, elapsed, playing))
            values.append(s.finish())
    return values


def generator_source(s, executable, counter=None):
    argv = [s.fixture(executable)]
    if counter:
        argv.append(str(counter))
    value = {'schema': 'xgc2.scene-source.v1', 'mode': 'random', 'format': 'geometry',
             'generator': {'command': argv, 'parameters': {'count': 60} if counter else {}}}
    return yaml.safe_dump(value)


def resolve_call(s, source, target, overrides=None):
    return s.call('generator.resolve', {'source': str(source), 'working_file': str(target),
                                       'overrides': overrides},
                  lambda: generation_module.resolve(str(source), str(target), overrides), effects=True)


def generation_scenarios():
    values = []
    with Scenario('generation', 'external-once', [case('test_scene_generation', 'external_generator_runs_once_and_saves_exact_parameters')]) as s:
        count = s.path('calls')
        source = s.initial_file('source.yaml', generator_source(s, 'generator_once.py', count))
        s.token(hashlib.sha256(source.read_bytes()).hexdigest(), '$sourceSha256')
        run1 = s.path('run1/scene.yaml')
        resolve_call(s, source, run1, {'count': 4})
        resolve_call(s, source, run1, {'count': 9})
        resolve_call(s, source, s.path('run2/scene.yaml'))
        resolve_call(s, source, s.path('run3/scene.yaml'), {'typo': 1})
        values.append(s.finish())
    with Scenario('generation', 'failure-no-publish', [case('test_scene_generation', 'generator_failure_does_not_publish_a_scene')]) as s:
        source = s.initial_file('source.yaml', generator_source(s, 'generator_failure.py'))
        s.token(hashlib.sha256(source.read_bytes()).hexdigest(), '$sourceSha256')
        resolve_call(s, source, s.path('result.yaml'))
        values.append(s.finish())
    with Scenario('generation', 'output-limit', [case('test_prepare', 'random_generator_output_is_bounded_before_world_commit')]) as s:
        source = s.initial_file('source.yaml', generator_source(s, 'generator_limit.py'))
        s.token(hashlib.sha256(source.read_bytes()).hexdigest(), '$sourceSha256')
        target = s.path('result.yaml')
        request = {'source': str(source), 'working_file': str(target), 'overrides': {}, 'max_bytes': 1048576}
        s.call('generator.resolve', request,
               lambda: generation_module.resolve(str(source), str(target), {}, max_bytes=1048576), effects=True)
        values.append(s.finish())
    return values


WORLD = ('<sdf version="1.6"><world name="fixture"><physics type="ode" name="physics">'
         '<max_step_size>0.001</max_step_size></physics><model name="authored"><static>true</static>'
         '<link name="body"><collision name="collision"><geometry><box><size>1 2 3</size></box>'
         '</geometry></collision></link></model></world></sdf>')


def prepare_fixture(s, mode='native'):
    asset = s.directory('asset')
    output = s.directory('run')
    s.initial_file('asset/source.world', WORLD)
    value = dict(scene(), id='fixture')
    s.initial_file('asset/scene.yaml', yaml.safe_dump(value))
    def attachment(name):
        return {'file': name, 'sha256': hashlib.sha256((asset / name).read_bytes()).hexdigest()}
    manifest = {'schemaVersion': 1, 'kind': 'xgc.scene-replay-asset.v1', 'revisionId': 'revision1',
                'simulators': {'gazebo': {'geometry': mode}}, 'gazeboWorld': attachment('source.world'),
                'sceneDocument': attachment('scene.yaml')}
    manifest_path = s.initial_file('asset/manifest.json', json.dumps(manifest))
    request = {'schema': 'xgc2.simulation.prepare.v1', 'configuration_revision': 4,
               'asset_directory': str(asset), 'expected_revision_id': 'revision1',
               'resource_root': str(s.root), 'socket_path': str(output / 'world.sock'), 'target_id': 'fixture',
               'output_grant': {'directory': str(output), 'max_bytes': 1048576}}
    return request, asset, output, manifest, manifest_path


def prepare_call(s, request):
    def execute():
        result = prepare_module.prepare(request)
        s.token(result['preparedWorldSha256'], '$preparedSha256')
        return result
    return s.call('gazebo.prepare', request, execute, effects=True)


def prepare_scenarios():
    values = []
    with Scenario('prepare', 'native-world', [case('test_prepare', 'native_world_keeps_exact_geometry_and_owns_only_its_granted_output')]) as s:
        request, _, _, _, _ = prepare_fixture(s)
        prepare_call(s, request)
        values.append(s.finish())
    with Scenario('prepare', 'document-and-media', [case('test_prepare', 'document_editability_and_media_attachments_are_provider_owned')]) as s:
        request, asset, _, manifest, manifest_path = prepare_fixture(s, 'document')
        manifest['simulators']['xsim'] = {'geometry': 'document'}
        for key, name in (('media', 'camera.png'), ('cameraInfo', 'calibration.yaml'), ('extrinsic', 'extrinsic.yaml')):
            path = s.initial_file('asset/' + name, 'opaque-owning-product-data')
            manifest[key] = {'file': name, 'sha256': hashlib.sha256(path.read_bytes()).hexdigest()}
        s.mutate(manifest_path, json.dumps(manifest))
        request.update(editable=True, parameters={'overrideWorldPhysicsTiming': True, 'maxStepSize': 0.002,
                                                  'realTimeUpdateRate': 0})
        prepare_call(s, request)
        values.append(s.finish())
    with Scenario('prepare', 'revision-hash-parameters-grant', [case('test_prepare', 'wrong_revision_bad_hash_unknown_physics_and_symlink_escape_are_rejected')]) as s:
        request, _, output, manifest, manifest_path = prepare_fixture(s)
        for override in ({'expected_revision_id': 'wrong'}, {'parameters': {'typo': 1}},
                         {'parameters': {'maxStepSize': True}}, {'configuration_revision': 4.0},
                         {'output_grant': {'directory': str(output), 'max_bytes': 1}}):
            prepare_call(s, dict(request, **override))
        manifest['gazeboWorld']['sha256'] = '0' * 64
        s.mutate(manifest_path, json.dumps(manifest))
        prepare_call(s, request)
        values.append(s.finish())
    with Scenario('prepare', 'random-output-limit', [case('test_prepare', 'random_generator_output_is_bounded_before_world_commit')]) as s:
        request, asset, _, manifest, manifest_path = prepare_fixture(s, 'document')
        source = asset / 'scene.yaml'
        s.mutate(source, generator_source(s, 'generator_limit.py'))
        source_digest = hashlib.sha256(source.read_bytes()).hexdigest()
        s.token(source_digest, '$sourceSha256')
        manifest['sceneDocument']['sha256'] = source_digest
        manifest['obstacleInput'] = {'mode': 'random', 'format': 'geometry'}
        s.mutate(manifest_path, json.dumps(manifest))
        request['run_id'] = 'run1'
        prepare_call(s, request)
        values.append(s.finish())
    with Scenario('prepare', 'explicit-rosters', [case('test_prepare', 'explicit_chassis_and_component_rosters_are_not_inferred')]) as s:
        request, _, _, _, _ = prepare_fixture(s)
        request['parameters'] = {'chassis_robot_ids': ['ugv1', 'ugv2'], 'required_components': ['vrpn']}
        prepare_call(s, request)
        for name in ('chassis_robot_ids', 'required_components'):
            # A Latin non-ASCII character exercises the same ASCII rejection
            # as the test's non-English roster, without adding that text.
            for roster in (['duplicate', 'duplicate'], ['robot-\u00e9'], ['../escape'],
                           list(map(str, range(17))), 'ugv1'):
                prepare_call(s, dict(request, parameters={name: roster}))
        values.append(s.finish())
    with Scenario('prepare', 'attachment-symlink', [case('test_prepare', 'asset_attachment_symlink_cannot_escape_its_asset_directory')]) as s:
        request, asset, _, _, _ = prepare_fixture(s)
        outside = s.initial_file('outside.world', WORLD)
        s.mutate(asset / 'source.world', symlink=outside)
        prepare_call(s, request)
        values.append(s.finish())
    return values


def assembly_input():
    moving = box('twist')
    moving['motion'] = {'type': 'constant_twist', 'linear': [2, 0, 0], 'angular': [0, 0, 0.3]}
    ping = box('ping')
    ping['motion'] = {'type': 'ping_pong', 'point_a': [1, 2, 3], 'point_b': [4, 2, 3], 'speed': 1}
    circle = varied_scene(1)['obstacles'][0]
    return {'request': {'fidelity': 'geometry-only', 'task': 'assembly-task', 'name': 'Placed Example',
                        'elements': [{'id': 'east', 'x': 5, 'y': -3, 'yaw': 0.7},
                                     {'id': 'west', 'x': -2, 'y': 4, 'yaw': -1.2}]},
            'documents': [yaml.safe_dump(scene(moving, ping, circle)), yaml.safe_dump(scene(box()))],
            'extrinsic': yaml.safe_dump({'parent_frame': 'world'})}


def assembly_call(s, value=None, raw=None):
    text = json.dumps(value, ensure_ascii=False) if raw is None else raw
    def execute():
        stdin = io.TextIOWrapper(io.BytesIO(text.encode('utf-8')), encoding='utf-8')
        stdout, stderr = io.StringIO(), io.StringIO()
        try:
            with mock.patch.object(sys, 'stdin', stdin), contextlib.redirect_stdout(stdout), contextlib.redirect_stderr(stderr):
                code = assembly_module.main()
            return {'exit_code': code, 'stdout': stdout.getvalue(), 'stderr': stderr.getvalue()}
        finally:
            stdin.close()
    return s.call('assembly.cli', {'stdin': text}, execute)


def assembly_scenarios():
    values = []
    names = ('placed-motion', 'fidelity', 'task', 'element-count', 'element-identity', 'placement',
             'frame-mismatch', 'camera-frame', 'result-name', 'document-validation', 'input-json', 'missing-fields')
    for name in names:
        with Scenario('assembly', name, ['assembly.py:main', 'assembly.py:assemble']) as s:
            value = assembly_input()
            if name == 'placed-motion':
                assembly_call(s, value)
                value['request']['fidelity'] = 'gazebo-world'
                assembly_call(s, value)
            elif name == 'fidelity':
                value['request']['fidelity'] = 'unknown'
                assembly_call(s, value)
            elif name == 'task':
                for task in ('../escape', 'x' * 65):
                    value['request']['task'] = task
                    assembly_call(s, value)
            elif name == 'element-count':
                value['request']['elements'] = value['request']['elements'][:1]
                assembly_call(s, value)
                value['request']['elements'] *= 2
                value['documents'] = value['documents'][:1]
                assembly_call(s, value)
            elif name == 'element-identity':
                for identity in ('east', 'with.dot', '../escape'):
                    value['request']['elements'][1]['id'] = identity
                    assembly_call(s, value)
            elif name == 'placement':
                for x in (10001, True, float('inf')):
                    value['request']['elements'][0]['x'] = x
                    assembly_call(s, value)
            elif name == 'frame-mismatch':
                second = dict(scene(box()), frame='another_world')
                value['documents'][1] = yaml.safe_dump(second)
                assembly_call(s, value)
            elif name == 'camera-frame':
                value['extrinsic'] = yaml.safe_dump({'parent_frame': 'camera'})
                assembly_call(s, value)
            elif name == 'result-name':
                value['request']['name'] = '../escape'
                assembly_call(s, value)
            elif name == 'document-validation':
                value['documents'][0] = 'schema: xgc2.scene.v1\nid: first\nid: second\nframe: world\nobstacles: []\n'
                assembly_call(s, value)
            elif name == 'input-json':
                for text in ('{', '{"request":{},"request":{}}', '{"request":NaN}'):
                    assembly_call(s, raw=text)
            else:
                assembly_call(s, {})
                value['request']['elements'][0].pop('id')
                assembly_call(s, value)
                assembly_call(s, None)
            values.append(s.finish())
    return values


def projection_document():
    # Plain authoring data from the projection test, without importing ROS stubs.
    norm = math.sqrt(0.3**2 + 0.5**2 + 0.2**2 + 0.79**2)
    general = [x / norm for x in (0.3, 0.5, 0.2, 0.79)]
    items = []
    for index in range(4):
        parts = []
        for k in range(12):
            a0, a1 = 2 * math.pi * k / 12, 2 * math.pi * (k + 1) / 12
            p0, p1 = (math.cos(a0), math.sin(a0), 0.0), (math.cos(a1), math.sin(a1), 0.0)
            half = math.atan2(p1[1] - p0[1], p1[0] - p0[0]) / 2
            orientation = (general if k % 3 == 2 else [0.0, math.sin(0.7), 0.0, math.cos(0.7)]
                           if k % 2 else [0.0, 0.0, math.sin(half), math.cos(half)])
            parts.append({'id': 'segment-{}'.format(k),
                          'pose': {'position': [(p0[0] + p1[0]) / 2, (p0[1] + p1[1]) / 2, 0.03 * k],
                                   'orientation': orientation},
                          'geometry': {'type': 'capsule', 'radius': 0.05,
                                       'height': 0.0 if k == 0 else math.dist(p0, p1)}})
        items.append({'id': 'ring-{}'.format(index), 'name': 'Ring {}'.format(index),
                      'pose': {'position': [index * 2.0, 1.0, 1.5]}, 'parts': parts})
    for index in range(6):
        item = {'id': 'pillar-{}'.format(index), 'name': 'Pillar "{}"'.format(index),
                'pose': {'position': [index * 1.5, -2.0, 1.0]},
                'parts': [{'id': 'body', 'geometry': {'type': 'cylinder', 'radius': 0.2, 'height': 4.0}},
                          {'id': 'cap', 'pose': {'position': [0, 0, 2.2]},
                           'geometry': {'type': 'sphere', 'radius': 0.25}},
                          {'id': 'base', 'geometry': {'type': 'box', 'size': [0.5, 0.5, 0.2]}}]}
        if index < 2:
            item['motion'] = {'type': 'constant_twist', 'linear': [0.1, 0.0, 0.0], 'angular': [0.0, 0.0, 0.3]}
        items.append(item)
    return dict(scene(*items), id='projection')


def projection_scenarios():
    with Scenario('document', 'envelope-json', [case('test_scene_projection', 'equals_json_dumps')]) as s:
        encoder = store_module.EnvelopeJson()
        document = projection_document()
        inputs = [{'epoch': 'e', 'revision': 3, 'dirty': False, 'sceneTime': 0.1,
                   'document': document, 'online': True}, {'document': document},
                  {'document': dict(document, obstacles=[])}, {'document': {'schema': 'x'}},
                  {'document': None, 'success': False, 'error': 'bad'}, {'success': True},
                  [1, 2.5, 'x'], 'text', {}, {'document': document, 1: 'non-string key'}]
        edited = dict(document, obstacles=list(document['obstacles']))
        edited['obstacles'][3] = dict(edited['obstacles'][3], name='renamed')
        del edited['obstacles'][5]
        inputs.extend(({'document': edited, 'revision': 4}, {'document': document}, {'document': edited},
                       {'document': document, 'sceneTime': float('nan')}))
        for value in inputs:
            s.call('envelope.encode', {'json': json.dumps(value, ensure_ascii=False)},
                   lambda value=value: encoder.dumps(value))
        return [s.finish()]


def wire_scenarios():
    from xgc2_xrpc import http as sdk_http
    from xgc2_xrpc.reference import ServiceRef
    from xgc2_xrpc.runtime import Runtime
    from xgc2_scene_runtime.xrpc_service import SceneService

    with Scenario('wire', 'authoring-http', [case('test_xrpc_control', 'real_authoring_host_and_retained_receipts')]) as s:
        store = s.create_store(scene())
        captures = []
        class CapturedBody(sdk_http.httpx.AsyncByteStream):
            def __init__(self, body):
                self.body = body
            async def __aiter__(self):
                yield self.body

        class CaptureTransport(sdk_http.httpx.AsyncHTTPTransport):
            # The SDK's public native transport seam still performs real UDS IO
            # and preserves its request trace. Capture before metadata checks,
            # since an instance fence deliberately makes the SDK refuse a reply.
            async def handle_async_request(self, request):
                response = await super().handle_async_request(request)
                try:
                    body = await response.aread()
                    headers = {name: response.headers[name] for name in (
                        'Content-Type', 'X-Xrpc-Instance-ID', 'X-Request-ID') if name in response.headers}
                    captures.append({'status': response.status_code, 'headers': headers,
                                     'body': json.loads(body), 'body_text': body.decode('utf-8')})
                    return sdk_http.httpx.Response(response.status_code, headers=response.headers,
                                                    stream=CapturedBody(body), extensions=response.extensions)
                finally:
                    await response.aclose()

        class Node:
            online = True
            def envelope(self):
                return store.envelope()
            def command_value(self, value):
                return store.command(value)
        node = Node()
        node.store = store
        socket_path = str(s.root / 'scene.sock')
        s.value['setup']['service'] = {'socket': socket_path, 'target_id': 'fixture', 'online': True}
        runtime = Runtime(blocking_workers=2, log_level='error')
        with contextlib.ExitStack() as stack:
            stack.callback(runtime.close)
            service = SceneService(node, path=socket_path, runtime=runtime, target_id='fixture')
            stack.callback(service.close)
            reference = service.start()
            s.token(reference['instance_id'], '$instance')
            s.value['setup']['service']['reference'] = reference
            def factory(**options):
                return CaptureTransport(uds=socket_path, **options)
            client = sdk_http.Client.from_service(ServiceRef.from_dict(reference), runtime=runtime,
                                                  local_target='fixture', transport_factory=factory)
            stack.callback(client.close)
            discovery = sdk_http.Client.from_service(ServiceRef.from_dict(dict(reference, instance_id='')),
                                                     runtime=runtime, local_target='fixture', discovery=True,
                                                     transport_factory=factory)
            stack.callback(discovery.close)

            def http_call(method, path, request_id, body=None, handle=client):
                headers = {'X-Request-ID': request_id}
                if handle.instance_id:
                    headers['X-Xrpc-Instance-ID'] = handle.instance_id
                if method == 'POST':
                    headers['Content-Type'] = 'application/json'
                request = {'method': method, 'path': path, 'headers': headers, 'body': body}
                def execute():
                    before = len(captures)
                    try:
                        result = handle.call(path, body, method=method, request_id=request_id)
                        outcome = {'disposition': result.disposition}
                    except (sdk_http.Fault, sdk_http.TransportError) as error:
                        outcome = {'type': type(error).__name__, 'message': str(error),
                                   'disposition': error.disposition}
                        if isinstance(error, sdk_http.Fault):
                            outcome.update(code=error.code, status=error.status)
                    if len(captures) != before + 1:
                        raise RuntimeError('HTTP capture missing for ' + path)
                    return dict(captures[-1], client=outcome)
                return s.call('http.call', request, execute)

            http_call('GET', '/v1/describe', 'describe', handle=discovery)
            http_call('GET', '/v1/describe?wait_ready_ms=10', 'describe-query', handle=discovery)
            http_call('GET', '/v1/health', 'health')
            http_call('GET', '/v1/scene', 'scene-before')
            command = {'operation': 'play', 'expectedEpoch': store.epoch, 'expectedRevision': 1}
            http_call('POST', '/v1/commands', 'native:play', command)
            http_call('POST', '/v1/commands/result', 'receipt', {'request_id': 'native:play'})
            http_call('POST', '/v1/commands', 'native:play', command)
            http_call('POST', '/v1/commands', 'native:play', dict(command, operation='pause'))
            http_call('POST', '/v1/commands', 'payload-id', dict(command, requestId='payload-id'))
            http_call('POST', '/v1/commands/result', 'unknown-receipt', {'request_id': 'unknown'})
            http_call('GET', '/v1/scene', 'scene-after')
            client.instance_id = 'stale'
            http_call('GET', '/v1/scene', 'instance-fence')
        # Runtime.close drains owned work; join its executor threads as well.
        runtime._executor.shutdown(wait=True)
        if Path(socket_path).exists():
            raise RuntimeError('Scene socket was not removed')
        s.value['cleanup'] = {'socket_exists': False, 'runtime_closed': runtime.closed}
        return [s.finish()]


EXCLUSIONS = {
    case('test_scene_frozen', 'scene_launch_forwards_the_frozen_argument_as_a_param'):
        'ROS-bound launch forwarding; to be written as a C++ launch contract test from the Python intent.',
    case('test_scene_scaling', 'gazebo_receives_one_batched_apply_per_revision'):
        'ROS-bound node/application batching; to be written as a C++ test from the Python intent.',
    case('test_scene_scaling', 'unchanged_consumer_heartbeats_do_not_republish_the_document'):
        'Dropped by decision: consumer registry and ROS-topic acknowledgement heartbeats.',
    case('test_scene_scaling', 'native_failure_and_recovery_change_the_published_view'):
        'ROS-bound published synchronization view; store/native-failure and store/lost-reply-resync retain native applied status. To be written as a C++ output test.',
    case('test_scene_scaling', 'tick_and_definition_scan_the_obstacle_list_a_constant_number_of_times'):
        'Performance-only obstacle scan counts (also ROS-bound).',
    store_case('one_edit_validates_and_emits_only_the_edited_obstacle'):
        'Performance-only validation/emission counts and Python object identity; to be written as a C++ immutability/cost test from the intent. Store undo and YAML behavior are recorded separately.',
}


def test_inventory(scene_root):
    inventory = {}
    for path in sorted((scene_root / 'test').glob('*.py')):
        for lineno, line in enumerate(path.read_text(encoding='utf-8').splitlines(), 1):
            match = re.match(r'\s*def (test_\w+)\(', line)
            if match:
                inventory[path.name + ':' + match.group(1)] = lineno
    return inventory


def coverage_rows(scene_root, scenarios):
    covered = {}
    for scenario in scenarios:
        for source in scenario['derived_from']:
            if ':test_' in source:
                covered.setdefault(source, []).append('corpus/' + scenario['scenario'] + '.json')
    inventory = test_inventory(scene_root)
    if set(covered) - set(inventory):
        raise RuntimeError('Unknown Python test references: ' + ', '.join(sorted(set(covered) - set(inventory))))
    rows = []
    for source, lineno in inventory.items():
        paths = covered.get(source, [])
        reason = EXCLUSIONS.get(source, '')
        if source.startswith('test_scene_consumers.py:'):
            reason = 'Dropped by decision: consumer registry and ROS-topic acknowledgement heartbeats.'
        if source == case('test_scene_projection', 'snapshots_markers_and_states_equal_the_uncached_projections'):
            reason = ('ROS-bound snapshots/markers/state projections are excluded; motion/rotating-twist '
                      'records only the plain motion sample. To be written as C++ ROS output tests.')
        if source == case('test_xrpc_control', 'simulation_waits_once_and_never_probes_entities'):
            reason = 'Simulation-client transport sequencing; to be written as a C++ client integration test from the Python intent.'
        if source == case('test_scene_projection', 'equals_json_dumps'):
            reason = ('Plain encoder cases only; consumer fields are dropped. JSON converts integer '
                      'object keys to strings; cache branch identity is a later C++ test concern.')
        if not paths and not reason:
            raise RuntimeError('Test is neither recorded nor explicitly excluded: ' + source)
        links = ', '.join('[{}]({})'.format(path[len('corpus/'):], path) for path in sorted(paths))
        coverage = links + ('; ' if links and reason else '') + reason
        rows.append('| `{}` | {} | {} |'.format(source, lineno, coverage))
    return rows


def record_all(directory):
    scenarios = []
    for build in (document_scenarios, store_scenarios, motion_scenarios, generation_scenarios,
                  prepare_scenarios, assembly_scenarios, projection_scenarios, wire_scenarios):
        scenarios.extend(build())
    names = set()
    total = 0
    for value in scenarios:
        name = value['scenario'] + '.json'
        if name in names:
            raise RuntimeError('Duplicate scenario: ' + name)
        names.add(name)
        body = (json.dumps(value, ensure_ascii=False, allow_nan=False, sort_keys=True,
                           separators=(',', ':')) + '\n').encode('utf-8')
        machine_paths = (str(HERE.parents[1]), str(Path(tempfile.gettempdir()) / 'scene-golden-'))
        if any(path.encode('utf-8') in body for path in machine_paths):
            raise RuntimeError('Unnormalized machine path in ' + name)
        path = directory / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(body)
        total += len(body)
    if total >= 3000000:
        raise RuntimeError('Corpus exceeds the 3 MB bound: {} bytes'.format(total))
    for path in directory.rglob('*.json'):
        if str(path.relative_to(directory)) not in names:
            path.unlink()
    return scenarios, total


def compare(left, right, label):
    def files(root):
        return {str(p.relative_to(root)): p.read_bytes() for p in root.rglob('*.json')}
    a, b = files(left), files(right)
    failures = []
    for name in sorted(set(a) | set(b)):
        if a.get(name) != b.get(name):
            detail = 'missing' if name not in b else 'unexpected' if name not in a else 'differs'
            failures.append('{}: {} {}'.format(label, name, detail))
    for failure in failures:
        print('FAIL ' + failure, file=sys.stderr)
    return not failures


def import_runtime(scene_root, xrpc_python):
    global document_module, store_module, motion_module, generation_module, prepare_module, assembly_module
    sys.path[:0] = [str(scene_root / 'src'), str(xrpc_python)]
    os.environ['PYTHONDONTWRITEBYTECODE'] = '1'
    from xgc2_scene_runtime import document as document_module
    from xgc2_scene_runtime import store as store_module
    from xgc2_scene_runtime import motion as motion_module
    from xgc2_scene_runtime import generation as generation_module
    from xgc2_scene_runtime import assembly as assembly_module
    from xgc2_scene_runtime.integrations.gazebo import prepare as prepare_module


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scene-root', type=Path, required=True,
                        help='Pinned runtime/scene_runtime directory (containing src and test)')
    parser.add_argument('--xrpc-python', type=Path, required=True, help='Read-only SDK Python directory')
    parser.add_argument('--check', action='store_true', help='Compare two fresh recordings and the committed corpus')
    args = parser.parse_args()
    started = time.monotonic()
    original_threads = set(threading.enumerate())
    import_runtime(args.scene_root.resolve(), args.xrpc_python.resolve())
    corpus = HERE / 'corpus'
    if args.check:
        with tempfile.TemporaryDirectory(prefix='scene-corpus-check-') as temporary:
            first, second = Path(temporary) / 'first', Path(temporary) / 'second'
            scenarios, total = record_all(first)
            record_all(second)
            deterministic = compare(first, second, 'repeat recording')
            committed = compare(first, corpus, 'committed corpus')
            rows = coverage_rows(args.scene_root, scenarios)
            readme = (HERE / 'README.md').read_text(encoding='utf-8')
            matrix = readme.split('<!-- coverage:start -->', 1)[1].split('<!-- coverage:end -->', 1)[0]
            expected = '\n| Python test case | Source line | Coverage or explicit exclusion |\n| --- | ---: | --- |\n' + '\n'.join(rows) + '\n'
            coverage = matrix == expected
            if not coverage:
                print('FAIL README coverage matrix differs from the pinned test inventory', file=sys.stderr)
            if not (deterministic and committed and coverage):
                return 1
    else:
        scenarios, total = record_all(corpus)
        rows = coverage_rows(args.scene_root, scenarios)
        # The format README is committed first; corpus recording fills its
        # inventory mechanically for the second, corpus/coverage commit.
        path = HERE / 'README.md'
        readme = path.read_text(encoding='utf-8')
        before, after = readme.split('<!-- coverage:start -->', 1)
        _, after = after.split('<!-- coverage:end -->', 1)
        table = '\n| Python test case | Source line | Coverage or explicit exclusion |\n| --- | ---: | --- |\n' + '\n'.join(rows) + '\n'
        path.write_text(before + '<!-- coverage:start -->' + table + '<!-- coverage:end -->' + after, encoding='utf-8')
    leaked = [thread.name for thread in threading.enumerate() if thread not in original_threads]
    if leaked:
        raise RuntimeError('Owned threads still running: ' + ', '.join(leaked))
    print('{}: {} scenarios, {} test cases, {} bytes, {:.2f}s'.format(
        'PASS' if args.check else 'Recorded', len(scenarios), len(rows), total, time.monotonic() - started))
    return 0


if __name__ == '__main__':
    sys.exit(main())
