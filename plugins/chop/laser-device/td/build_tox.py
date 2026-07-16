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
# platform binary next to the .toe file.
PLUGIN_PATH_EXPR = (
    "project.folder + ('/laser_device.plugin' if app.osName == 'MacOS' "
    "else '/laser_device.dll')"
)

# (comp par name, inner par name, appender, kwargs)
PROMOTED_PARS = [
    ('Active', 'Active', 'appendToggle', {}),
    ('Backend', 'Backend', 'appendMenu', {}),
    ('Device', 'Device', 'appendStrMenu', {}),
    ('Refreshdevices', 'Refresh', 'appendPulse', {}),
    ('Pps', 'Pps', 'appendInt', {}),
    ('Intensity', 'Intensity', 'appendFloat', {}),
    ('Scale', 'Scale', 'appendFloat', {}),
    ('Defaultcolor', 'Defaultcolor', 'appendRGBA', {}),
    ('Address', 'Address', 'appendStr', {}),
    ('Sendername', 'Sendername', 'appendStr', {}),
]


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
    for comp_name, inner_name, appender, kwargs in PROMOTED_PARS:
        try:
            new_pars = getattr(page, appender)(comp_name, **kwargs)
        except Exception as e:  # noqa: BLE001 - report and continue
            print(f'skipped {comp_name}: {e}')
            continue

        inner = getattr(cpp.par, inner_name, None)
        if inner is None:
            # Plugin binary not found at build time: parameters exist on the
            # COMP but stay unbound. Re-run this script with the binary in
            # place to bind them.
            print(f'plugin par {inner_name} missing; {comp_name} left unbound')
            continue

        if inner.isMenu:
            new_pars[0].menuSource = f"op('{cpp.path}').par.{inner_name}"
        members = [inner] if inner.tupletName is None else inner.tuplet
        for i, member in enumerate(members):
            member.bindExpr = f"parent().par.{new_pars[i].name}"

    comp.par.opshortcut = TOX_NAME
    comp.comment = (
        'Laser Device (td-rs): streams x/y/r/g/b CHOP channels to laser DAC '
        'hardware or PONK network receivers. Requires laser_device.plugin/.dll '
        'next to the .toe file.'
    )

    tox_path = f'{project.folder}/{TOX_NAME}.tox'
    comp.save(tox_path)
    print(f'saved {tox_path}')
    return comp


build()
