# Builds laser_device.tox: a reusable Base COMP wrapping the Laser Device
# plugin via a stock CPlusPlus CHOP, so end users can drag one .tox into a
# project without installing the plugin globally.
#
# Run inside TouchDesigner's textport with the compiled plugin binary sitting
# next to your .toe (see td/README.md for the expected layout):
#
#   exec(open('/path/to/td-rs/plugins/chop/laser-device/td/build_tox.py').read())
#
# The wrapper resolves the platform binary at load time, promotes the
# plugin's parameters to the COMP, and exposes In/Out CHOP connectors.

TOX_NAME = 'laser_device'

# Plugin path expression evaluated in the user's project: looks for the
# platform binary next to the .toe file. NOTE the names differ per platform
# because td-rs-xtask normalizes them differently: the macOS bundle is
# laser_device.plugin (hyphens replaced) while the Windows build keeps the
# package name, laser-device.dll.
PLUGIN_PATH_EXPR = (
    "project.folder + ('/laser_device.plugin' if app.osName == 'MacOS' "
    "else '/laser-device.dll')"
)

# (comp par name, inner par name, appender)
PROMOTED_PARS = [
    ('Active', 'Active', 'appendToggle'),
    ('Backend', 'Backend', 'appendMenu'),
    ('Device', 'Device', 'appendStrMenu'),
    ('Refreshdevices', 'Refresh', 'appendPulse'),
    ('Pps', 'Pps', 'appendInt'),
    ('Intensity', 'Intensity', 'appendFloat'),
    ('Scale', 'Scale', 'appendFloat'),
    ('Defaultcolor', 'Defaultcolor', 'appendRGBA'),
    ('Address', 'Address', 'appendStr'),
    ('Sendername', 'Sendername', 'appendStr'),
]

# The plugin reacts to Refresh via the SDK's pulsePressed event, which a
# value binding does not deliver — forward the promoted pulse explicitly.
# Guarded so a missing plugin binary flags nothing instead of erroring.
REFRESH_CALLBACK = '''\
def onPulse(par):
    if par.name != 'Refreshdevices':
        return
    laser = op('laser')
    refresh = getattr(laser.par, 'Refresh', None) if laser else None
    if refresh is not None:
        refresh.pulse()
    return
'''


def plugin_pars(cpp, name):
    """All Par members for a plugin parameter.

    Multi-value parameters (the RGBA Defaultcolor) expose only suffixed
    member Pars (Defaultcolorr/g/b/a) — there is no Par attribute with the
    bare tuplet name, so fall back to a tuplet-name match.
    """
    par = getattr(cpp.par, name, None)
    if par is not None:
        return list(par.tuplet)
    return [p for p in cpp.pars(name + '*') if p.tupletName == name]


def build():
    root = op('/')
    existing = root.op(TOX_NAME)
    if existing:
        existing.destroy()
    comp = root.create(baseCOMP, TOX_NAME)

    in1 = comp.create(inCHOP, 'in1')
    cpp = comp.create(cplusplusCHOP, 'laser')
    out1 = comp.create(outCHOP, 'out1')
    in1.nodeX, in1.nodeY = -200, 0
    cpp.nodeX, cpp.nodeY = 0, 0
    out1.nodeX, out1.nodeY = 200, 0

    cpp.par.plugin.expr = PLUGIN_PATH_EXPR
    cpp.inputConnectors[0].connect(in1)
    out1.inputConnectors[0].connect(cpp)

    page = comp.appendCustomPage('Laser')
    for comp_name, inner_name, appender in PROMOTED_PARS:
        try:
            new_pars = getattr(page, appender)(comp_name)
        except Exception as e:  # noqa: BLE001 - report and continue
            print(f'skipped {comp_name}: {e}')
            continue

        if appender == 'appendPulse':
            # Pulses are forwarded by the parexec DAT below, not bound.
            continue

        members = plugin_pars(cpp, inner_name)
        if not members:
            # Plugin binary not found at build time: parameters exist on the
            # COMP but stay unbound. Re-run this script with the binary in
            # place to bind them.
            print(f'plugin par {inner_name} missing; {comp_name} left unbound')
            continue

        if members[0].isMenu:
            # Relative reference so the .tox survives renames/relocation.
            new_pars[0].menuSource = f"op('./laser').par.{members[0].name}"
        for i, member in enumerate(members):
            # Match tuplet members by name suffix (Defaultcolorr -> ...r):
            # pars() ordering is undocumented, so index alignment alone could
            # cross-bind color channels.
            suffix = member.name[len(inner_name):]
            target = next(
                (p for p in new_pars if p.name[len(comp_name):] == suffix),
                new_pars[i] if i < len(new_pars) else None,
            )
            if target is not None:
                # Seed the COMP par with the plugin's default before binding
                # (bound pars adopt the bind master's value — without this a
                # fresh .tox would start with Pps=0, Intensity=0, ...).
                try:
                    target.val = member.eval()
                except Exception as e:  # noqa: BLE001
                    print(f'could not seed {target.name}: {e}')
                member.bindExpr = f"parent().par.{target.name}"

    # Forward the promoted Refresh pulse as a real pulse event.
    parexec = comp.create(parameterexecuteDAT, 'refresh_exec')
    parexec.par.op = '..'
    parexec.par.pars = 'Refreshdevices'
    # Pin the toggles the callback depends on rather than trusting the DAT's
    # defaults ('On Pulse' and 'Custom' must be on for a custom pulse par).
    for toggle in ('active', 'custom', 'onpulse'):
        toggle_par = getattr(parexec.par, toggle, None)
        if toggle_par is not None:
            toggle_par.val = True
    parexec.text = REFRESH_CALLBACK

    comp.par.opshortcut = TOX_NAME
    comp.comment = (
        'Laser Device (td-rs): streams x/y/r/g/b CHOP channels to laser DAC '
        'hardware or PONK network receivers. Requires the platform plugin '
        'binary next to the .toe file (see td/README.md).'
    )

    tox_path = f'{project.folder}/{TOX_NAME}.tox'
    comp.save(tox_path)
    print(f'saved {tox_path}')
    return comp


build()
