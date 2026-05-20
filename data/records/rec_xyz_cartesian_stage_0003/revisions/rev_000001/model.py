from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports. If the model needs mesh assets, create an
# `AssetContext` inside the editable section.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

ASSETS = AssetContext.from_script(__file__)

OUTRIGGER_LENGTH = 0.72
OUTRIGGER_WIDTH = 0.06
OUTRIGGER_HEIGHT = 0.03
OUTRIGGER_Y = 0.15

DECK_LENGTH = 0.62
DECK_WIDTH = 0.24
DECK_HEIGHT = 0.024
DECK_BOTTOM = 0.044
DECK_TOP = DECK_BOTTOM + DECK_HEIGHT

X_RAIL_LENGTH = 0.54
X_RAIL_WIDTH = 0.020
X_RAIL_HEIGHT = 0.014
X_RAIL_Y = 0.076
X_RAIL_TOP = DECK_TOP + X_RAIL_HEIGHT
X_TRAVEL = 0.16

X_GUARD_WIDTH = 0.018
X_GUARD_HEIGHT = 0.022
X_GUARD_Y = 0.122

X_BEARING_X = 0.055
X_BEARING_LENGTH = 0.045
X_BEARING_WIDTH = 0.034
X_BEARING_HEIGHT = 0.020

X_BRIDGE_LENGTH = 0.18
X_BRIDGE_WIDTH = 0.30
X_BRIDGE_THICKNESS = 0.018
X_BRIDGE_BOTTOM = X_BEARING_HEIGHT
X_BRIDGE_TOP = X_BRIDGE_BOTTOM + X_BRIDGE_THICKNESS

Y_BEAM_LENGTH = 0.34
Y_BEAM_WIDTH = 0.11
Y_BEAM_THICKNESS = 0.022
Y_BEAM_BOTTOM = X_BRIDGE_TOP
Y_BEAM_TOP = Y_BEAM_BOTTOM + Y_BEAM_THICKNESS

Y_RAIL_LENGTH = 0.30
Y_RAIL_WIDTH = 0.018
Y_RAIL_HEIGHT = 0.012
Y_RAIL_X = 0.035
Y_RAIL_TOP = Y_BEAM_TOP + Y_RAIL_HEIGHT
Y_TRAVEL = 0.09

Y_GUARD_WIDTH = 0.014
Y_GUARD_HEIGHT = 0.018
Y_GUARD_X = 0.064

Y_BEARING_Y = 0.060
Y_BEARING_LENGTH = 0.032
Y_BEARING_WIDTH = 0.045
Y_BEARING_HEIGHT = 0.018

Y_PLATE_LENGTH = 0.17
Y_PLATE_WIDTH = 0.20
Y_PLATE_THICKNESS = 0.018
Y_PLATE_BOTTOM = Y_BEARING_HEIGHT
Y_PLATE_TOP = Y_PLATE_BOTTOM + Y_PLATE_THICKNESS

Z_COLUMN_WIDTH = 0.14
Z_COLUMN_DEPTH = 0.040
Z_COLUMN_Y = 0.020
Z_COLUMN_HEIGHT = 0.26
Z_COLUMN_BOTTOM = 0.052
Z_COLUMN_TOP = Z_COLUMN_BOTTOM + Z_COLUMN_HEIGHT

Z_RAIL_WIDTH = 0.018
Z_RAIL_DEPTH = 0.012
Z_RAIL_X = 0.040
Z_RAIL_HEIGHT = 0.24
Z_RAIL_BOTTOM = Z_COLUMN_BOTTOM
Z_RAIL_CENTER_Y = Z_COLUMN_Y + Z_COLUMN_DEPTH / 2.0 + Z_RAIL_DEPTH / 2.0
Z_RAIL_FRONT_Y = Z_COLUMN_Y + Z_COLUMN_DEPTH / 2.0 + Z_RAIL_DEPTH
Z_TRAVEL = 0.055

Z_GUARD_WIDTH = 0.012
Z_GUARD_DEPTH = 0.014
Z_GUARD_X = 0.066
Z_GUARD_CENTER_Y = Z_COLUMN_Y + Z_COLUMN_DEPTH / 2.0 + Z_GUARD_DEPTH / 2.0

Z_BEARING_X = Z_RAIL_X
Z_BEARING_WIDTH = 0.030
Z_BEARING_DEPTH = 0.028
Z_BEARING_HEIGHT = 0.035
Z_BEARING_Z = 0.044

Z_CARRIAGE_WIDTH = 0.14
Z_CARRIAGE_DEPTH = 0.018
Z_CARRIAGE_HEIGHT = 0.16
Z_CARRIAGE_BACK_Y = Z_BEARING_DEPTH

TOOL_PLATE_WIDTH = 0.11
TOOL_PLATE_DEPTH = 0.010
TOOL_PLATE_HEIGHT = 0.12
TOOL_PLATE_BACK_Y = 0.058


def _box(
    length: float,
    width: float,
    height: float,
    *,
    x: float = 0.0,
    y: float = 0.0,
    z: float = 0.0,
    chamfer: float | None = None,
) -> cq.Workplane:
    solid = cq.Workplane("XY").box(length, width, height, centered=(True, True, False)).translate((x, y, z))
    if chamfer and chamfer > 0.0:
        solid = solid.edges("|Z").chamfer(chamfer)
    return solid


def _union_all(*solids: cq.Workplane) -> cq.Workplane:
    result = solids[0]
    for solid in solids[1:]:
        result = result.union(solid)
    return result


def _add_mesh_visual(
    part,
    shape: cq.Workplane,
    *,
    filename: str,
    name: str,
    material,
) -> None:
    part.visual(
        mesh_from_cadquery(shape, filename, assets=ASSETS),
        material=material,
        name=name,
    )


def _make_base_body() -> cq.Workplane:
    deck = _box(DECK_LENGTH, DECK_WIDTH, DECK_HEIGHT, z=DECK_BOTTOM, chamfer=0.003)
    for pocket_y in (-0.102, 0.102):
        deck = deck.cut(_box(0.24, 0.032, 0.006, y=pocket_y, z=DECK_TOP - 0.006))
    deck = deck.cut(_box(0.12, 0.08, 0.012, z=DECK_TOP - 0.012))

    left_outrigger = _box(
        OUTRIGGER_LENGTH,
        OUTRIGGER_WIDTH,
        OUTRIGGER_HEIGHT,
        y=OUTRIGGER_Y,
        z=0.0,
        chamfer=0.003,
    )
    right_outrigger = _box(
        OUTRIGGER_LENGTH,
        OUTRIGGER_WIDTH,
        OUTRIGGER_HEIGHT,
        y=-OUTRIGGER_Y,
        z=0.0,
        chamfer=0.003,
    )

    pedestals = [
        _box(0.08, 0.05, 0.022, x=x, y=y, z=OUTRIGGER_HEIGHT, chamfer=0.0015)
        for x in (-0.19, 0.19)
        for y in (-0.095, 0.095)
    ]
    end_web_pos = _box(0.016, 0.26, 0.05, x=0.28, z=OUTRIGGER_HEIGHT, chamfer=0.0015)
    end_web_neg = _box(0.016, 0.26, 0.05, x=-0.28, z=OUTRIGGER_HEIGHT, chamfer=0.0015)

    return _union_all(deck, left_outrigger, right_outrigger, *pedestals, end_web_pos, end_web_neg)


def _make_x_carriage_body() -> cq.Workplane:
    bridge = _box(
        X_BRIDGE_LENGTH,
        X_BRIDGE_WIDTH,
        X_BRIDGE_THICKNESS,
        z=X_BRIDGE_BOTTOM,
        chamfer=0.0025,
    )
    bridge = bridge.cut(_box(0.10, 0.14, 0.012, z=X_BRIDGE_BOTTOM + 0.003))

    y_beam = _box(
        Y_BEAM_WIDTH,
        Y_BEAM_LENGTH,
        Y_BEAM_THICKNESS,
        z=Y_BEAM_BOTTOM,
        chamfer=0.002,
    )
    y_beam = y_beam.cut(_box(0.07, 0.16, 0.005, z=Y_BEAM_TOP - 0.005))

    left_web = _box(0.014, 0.22, Y_BEAM_BOTTOM - X_BRIDGE_BOTTOM, x=-0.040, z=X_BRIDGE_BOTTOM, chamfer=0.001)
    right_web = _box(0.014, 0.22, Y_BEAM_BOTTOM - X_BRIDGE_BOTTOM, x=0.040, z=X_BRIDGE_BOTTOM, chamfer=0.001)

    return _union_all(bridge, y_beam, left_web, right_web)


def _make_bearing_block(
    length: float,
    width: float,
    height: float,
    *,
    x: float = 0.0,
    y: float = 0.0,
    z: float = 0.0,
) -> cq.Workplane:
    base_height = height * 0.55
    cap_height = height - base_height
    block = _box(length, width, base_height, x=x, y=y, z=z, chamfer=min(length, width) * 0.03)
    block = block.union(
        _box(
            length * 0.86,
            width * 0.82,
            cap_height,
            x=x,
            y=y,
            z=z + base_height,
            chamfer=min(length, width) * 0.025,
        )
    )
    block = block.cut(
        _box(
            length * 0.34,
            width * 0.48,
            cap_height * 0.45,
            x=x,
            y=y,
            z=z + height - cap_height * 0.45,
        )
    )
    return block


def _make_x_bearing_pair(y_pos: float) -> cq.Workplane:
    first = _make_bearing_block(
        X_BEARING_LENGTH,
        X_BEARING_WIDTH,
        X_BEARING_HEIGHT,
        x=-X_BEARING_X,
        y=y_pos,
        z=0.0,
    )
    second = _make_bearing_block(
        X_BEARING_LENGTH,
        X_BEARING_WIDTH,
        X_BEARING_HEIGHT,
        x=X_BEARING_X,
        y=y_pos,
        z=0.0,
    )
    return _union_all(first, second)


def _make_y_carriage_body() -> cq.Workplane:
    plate = _box(
        Y_PLATE_LENGTH,
        Y_PLATE_WIDTH,
        Y_PLATE_THICKNESS,
        z=Y_PLATE_BOTTOM,
        chamfer=0.002,
    )
    plate = plate.cut(_box(0.06, 0.10, 0.010, z=Y_PLATE_BOTTOM + 0.004))

    column_base = _box(0.12, 0.10, 0.016, z=Y_PLATE_TOP, chamfer=0.0015)
    column = _box(
        Z_COLUMN_WIDTH,
        Z_COLUMN_DEPTH,
        Z_COLUMN_HEIGHT,
        y=Z_COLUMN_Y,
        z=Z_COLUMN_BOTTOM,
        chamfer=0.002,
    )
    column = column.cut(_box(0.09, 0.006, 0.18, y=0.003, z=0.09))

    left_gusset = _box(0.018, 0.050, Z_COLUMN_BOTTOM - Y_PLATE_TOP, x=-0.046, y=0.006, z=Y_PLATE_TOP, chamfer=0.001)
    right_gusset = _box(0.018, 0.050, Z_COLUMN_BOTTOM - Y_PLATE_TOP, x=0.046, y=0.006, z=Y_PLATE_TOP, chamfer=0.001)

    return _union_all(plate, column_base, column, left_gusset, right_gusset)


def _make_y_bearing_pair(x_pos: float) -> cq.Workplane:
    first = _make_bearing_block(
        Y_BEARING_LENGTH,
        Y_BEARING_WIDTH,
        Y_BEARING_HEIGHT,
        x=x_pos,
        y=-Y_BEARING_Y,
        z=0.0,
    )
    second = _make_bearing_block(
        Y_BEARING_LENGTH,
        Y_BEARING_WIDTH,
        Y_BEARING_HEIGHT,
        x=x_pos,
        y=Y_BEARING_Y,
        z=0.0,
    )
    return _union_all(first, second)


def _make_z_carriage_body() -> cq.Workplane:
    carriage = _box(
        Z_CARRIAGE_WIDTH,
        Z_CARRIAGE_DEPTH,
        Z_CARRIAGE_HEIGHT,
        y=Z_CARRIAGE_BACK_Y + Z_CARRIAGE_DEPTH / 2.0,
        z=-Z_CARRIAGE_HEIGHT / 2.0,
        chamfer=0.0015,
    )
    carriage = carriage.cut(
        _box(
            0.06,
            Z_CARRIAGE_DEPTH + 0.002,
            0.08,
            y=Z_CARRIAGE_BACK_Y + Z_CARRIAGE_DEPTH / 2.0,
            z=-0.04,
        )
    )

    central_boss = _box(0.08, 0.012, 0.08, y=0.052, z=-0.04, chamfer=0.001)
    upper_rib = _box(0.10, 0.016, 0.018, y=0.050, z=0.055, chamfer=0.001)
    lower_rib = _box(0.10, 0.016, 0.018, y=0.050, z=-0.073, chamfer=0.001)

    return _union_all(carriage, central_boss, upper_rib, lower_rib)


def _make_tool_plate() -> cq.Workplane:
    center_y = TOOL_PLATE_BACK_Y + TOOL_PLATE_DEPTH / 2.0
    plate = _box(
        TOOL_PLATE_WIDTH,
        TOOL_PLATE_DEPTH,
        TOOL_PLATE_HEIGHT,
        y=center_y,
        z=-TOOL_PLATE_HEIGHT / 2.0,
        chamfer=0.0012,
    )
    plate = plate.cut(
        _box(
            0.020,
            TOOL_PLATE_DEPTH + 0.002,
            0.030,
            x=-0.030,
            y=center_y,
            z=0.018,
        )
    )
    plate = plate.cut(
        _box(
            0.020,
            TOOL_PLATE_DEPTH + 0.002,
            0.030,
            x=0.030,
            y=center_y,
            z=0.018,
        )
    )
    plate = plate.cut(
        _box(
            0.056,
            TOOL_PLATE_DEPTH + 0.002,
            0.020,
            y=center_y,
            z=-0.052,
        )
    )
    datum_land = _box(0.050, TOOL_PLATE_DEPTH * 0.7, 0.010, y=center_y, z=-0.005, chamfer=0.0008)
    return _union_all(plate, datum_land)


def _make_z_bearing_pair(x_pos: float) -> cq.Workplane:
    lower = _make_bearing_block(
        Z_BEARING_WIDTH,
        Z_BEARING_DEPTH,
        Z_BEARING_HEIGHT,
        x=x_pos,
        y=Z_BEARING_DEPTH / 2.0,
        z=-Z_BEARING_Z - Z_BEARING_HEIGHT / 2.0,
    )
    upper = _make_bearing_block(
        Z_BEARING_WIDTH,
        Z_BEARING_DEPTH,
        Z_BEARING_HEIGHT,
        x=x_pos,
        y=Z_BEARING_DEPTH / 2.0,
        z=Z_BEARING_Z - Z_BEARING_HEIGHT / 2.0,
    )
    return _union_all(lower, upper)


def _check_prismatic_motion(
    ctx: TestContext,
    *,
    articulation,
    child,
    axis: str,
    lower: float,
    upper: float,
    name: str,
) -> bool:
    axis_index = {"x": 0, "y": 1, "z": 2}[axis]

    with ctx.pose({articulation: lower}):
        lower_pos = ctx.part_world_position(child)
    with ctx.pose({articulation: upper}):
        upper_pos = ctx.part_world_position(child)

    if lower_pos is None or upper_pos is None:
        return ctx.fail(name, "part world position unavailable")

    delta = [upper_pos[i] - lower_pos[i] for i in range(3)]
    expected = upper - lower
    primary_error = abs(delta[axis_index] - expected)
    orthogonal_error = max(abs(delta[i]) for i in range(3) if i != axis_index)

    return ctx.check(
        name,
        primary_error <= 1e-6 and orthogonal_error <= 1e-6,
        (
            f"delta={tuple(round(value, 6) for value in delta)}, "
            f"expected {expected:.6f} along {axis}"
        ),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="xyz_cartesian_stage", assets=ASSETS)

    structure = model.material("structure_grey", rgba=(0.46, 0.48, 0.51, 1.0))
    rail_steel = model.material("rail_steel", rgba=(0.28, 0.30, 0.33, 1.0))
    bearing_black = model.material("bearing_black", rgba=(0.13, 0.14, 0.15, 1.0))
    cover_grey = model.material("cover_grey", rgba=(0.67, 0.69, 0.72, 1.0))
    guard_dark = model.material("guard_dark", rgba=(0.19, 0.20, 0.22, 1.0))
    datum_metal = model.material("datum_metal", rgba=(0.80, 0.82, 0.84, 1.0))

    base = model.part("base_frame")
    _add_mesh_visual(
        base,
        _make_base_body(),
        filename="base_frame_body.obj",
        name="base_body",
        material=structure,
    )
    base.visual(
        Box((X_RAIL_LENGTH, X_RAIL_WIDTH, X_RAIL_HEIGHT)),
        origin=Origin(xyz=(0.0, X_RAIL_Y, DECK_TOP + X_RAIL_HEIGHT / 2.0)),
        material=rail_steel,
        name="x_left_rail",
    )
    base.visual(
        Box((X_RAIL_LENGTH, X_RAIL_WIDTH, X_RAIL_HEIGHT)),
        origin=Origin(xyz=(0.0, -X_RAIL_Y, DECK_TOP + X_RAIL_HEIGHT / 2.0)),
        material=rail_steel,
        name="x_right_rail",
    )
    base.visual(
        Box((X_RAIL_LENGTH, X_GUARD_WIDTH, X_GUARD_HEIGHT)),
        origin=Origin(xyz=(0.0, X_GUARD_Y, DECK_TOP + X_GUARD_HEIGHT / 2.0)),
        material=guard_dark,
        name="x_left_guard",
    )
    base.visual(
        Box((X_RAIL_LENGTH, X_GUARD_WIDTH, X_GUARD_HEIGHT)),
        origin=Origin(xyz=(0.0, -X_GUARD_Y, DECK_TOP + X_GUARD_HEIGHT / 2.0)),
        material=guard_dark,
        name="x_right_guard",
    )
    base.visual(
        Box((0.24, 0.032, 0.006)),
        origin=Origin(xyz=(0.0, 0.102, DECK_TOP - 0.003)),
        material=cover_grey,
        name="base_left_cover",
    )
    base.visual(
        Box((0.24, 0.032, 0.006)),
        origin=Origin(xyz=(0.0, -0.102, DECK_TOP - 0.003)),
        material=cover_grey,
        name="base_right_cover",
    )
    base.visual(
        Box((0.07, 0.045, 0.006)),
        origin=Origin(xyz=(-0.22, -0.102, DECK_TOP + 0.003)),
        material=datum_metal,
        name="base_datum_pad",
    )
    base.inertial = Inertial.from_geometry(
        Box((OUTRIGGER_LENGTH, 0.36, 0.09)),
        mass=28.0,
        origin=Origin(xyz=(0.0, 0.0, 0.045)),
    )

    x_carriage = model.part("x_carriage")
    _add_mesh_visual(
        x_carriage,
        _make_x_carriage_body(),
        filename="x_carriage_body.obj",
        name="x_carriage_body",
        material=structure,
    )
    _add_mesh_visual(
        x_carriage,
        _make_x_bearing_pair(X_RAIL_Y),
        filename="x_left_bearings.obj",
        name="x_left_bearings",
        material=bearing_black,
    )
    _add_mesh_visual(
        x_carriage,
        _make_x_bearing_pair(-X_RAIL_Y),
        filename="x_right_bearings.obj",
        name="x_right_bearings",
        material=bearing_black,
    )
    x_carriage.visual(
        Box((Y_RAIL_WIDTH, Y_RAIL_LENGTH, Y_RAIL_HEIGHT)),
        origin=Origin(xyz=(Y_RAIL_X, 0.0, Y_BEAM_TOP + Y_RAIL_HEIGHT / 2.0)),
        material=rail_steel,
        name="y_left_rail",
    )
    x_carriage.visual(
        Box((Y_RAIL_WIDTH, Y_RAIL_LENGTH, Y_RAIL_HEIGHT)),
        origin=Origin(xyz=(-Y_RAIL_X, 0.0, Y_BEAM_TOP + Y_RAIL_HEIGHT / 2.0)),
        material=rail_steel,
        name="y_right_rail",
    )
    x_carriage.visual(
        Box((Y_GUARD_WIDTH, Y_RAIL_LENGTH, Y_GUARD_HEIGHT)),
        origin=Origin(xyz=(Y_GUARD_X, 0.0, Y_BEAM_TOP + Y_GUARD_HEIGHT / 2.0)),
        material=guard_dark,
        name="y_left_guard",
    )
    x_carriage.visual(
        Box((Y_GUARD_WIDTH, Y_RAIL_LENGTH, Y_GUARD_HEIGHT)),
        origin=Origin(xyz=(-Y_GUARD_X, 0.0, Y_BEAM_TOP + Y_GUARD_HEIGHT / 2.0)),
        material=guard_dark,
        name="y_right_guard",
    )
    x_carriage.visual(
        Box((0.004, Y_RAIL_LENGTH, Y_GUARD_HEIGHT)),
        origin=Origin(xyz=(0.056, 0.0, Y_BEAM_TOP + Y_GUARD_HEIGHT / 2.0)),
        material=structure,
        name="y_left_guard_bracket",
    )
    x_carriage.visual(
        Box((0.004, Y_RAIL_LENGTH, Y_GUARD_HEIGHT)),
        origin=Origin(xyz=(-0.056, 0.0, Y_BEAM_TOP + Y_GUARD_HEIGHT / 2.0)),
        material=structure,
        name="y_right_guard_bracket",
    )
    x_carriage.visual(
        Box((0.07, 0.16, 0.005)),
        origin=Origin(xyz=(0.0, 0.0, Y_BEAM_TOP - 0.0025)),
        material=cover_grey,
        name="x_access_cover",
    )
    x_carriage.inertial = Inertial.from_geometry(
        Box((0.18, 0.34, 0.08)),
        mass=7.5,
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
    )

    y_carriage = model.part("y_carriage")
    _add_mesh_visual(
        y_carriage,
        _make_y_carriage_body(),
        filename="y_carriage_body.obj",
        name="y_carriage_body",
        material=structure,
    )
    _add_mesh_visual(
        y_carriage,
        _make_y_bearing_pair(Y_RAIL_X),
        filename="y_left_bearings.obj",
        name="y_left_bearings",
        material=bearing_black,
    )
    _add_mesh_visual(
        y_carriage,
        _make_y_bearing_pair(-Y_RAIL_X),
        filename="y_right_bearings.obj",
        name="y_right_bearings",
        material=bearing_black,
    )
    y_carriage.visual(
        Box((Z_RAIL_WIDTH, Z_RAIL_DEPTH, Z_RAIL_HEIGHT)),
        origin=Origin(
            xyz=(
                Z_RAIL_X,
                Z_RAIL_CENTER_Y,
                Z_RAIL_BOTTOM + Z_RAIL_HEIGHT / 2.0,
            )
        ),
        material=rail_steel,
        name="z_left_rail",
    )
    y_carriage.visual(
        Box((Z_RAIL_WIDTH, Z_RAIL_DEPTH, Z_RAIL_HEIGHT)),
        origin=Origin(
            xyz=(
                -Z_RAIL_X,
                Z_RAIL_CENTER_Y,
                Z_RAIL_BOTTOM + Z_RAIL_HEIGHT / 2.0,
            )
        ),
        material=rail_steel,
        name="z_right_rail",
    )
    y_carriage.visual(
        Box((Z_GUARD_WIDTH, Z_GUARD_DEPTH, Z_RAIL_HEIGHT + 0.012)),
        origin=Origin(
            xyz=(
                Z_GUARD_X,
                Z_GUARD_CENTER_Y,
                Z_RAIL_BOTTOM + (Z_RAIL_HEIGHT + 0.012) / 2.0,
            )
        ),
        material=guard_dark,
        name="z_left_guard",
    )
    y_carriage.visual(
        Box((Z_GUARD_WIDTH, Z_GUARD_DEPTH, Z_RAIL_HEIGHT + 0.012)),
        origin=Origin(
            xyz=(
                -Z_GUARD_X,
                Z_GUARD_CENTER_Y,
                Z_RAIL_BOTTOM + (Z_RAIL_HEIGHT + 0.012) / 2.0,
            )
        ),
        material=guard_dark,
        name="z_right_guard",
    )
    y_carriage.visual(
        Box((0.09, 0.006, 0.18)),
        origin=Origin(xyz=(0.0, 0.003, 0.18)),
        material=cover_grey,
        name="z_access_cover",
    )
    y_carriage.inertial = Inertial.from_geometry(
        Box((0.17, 0.20, 0.32)),
        mass=9.0,
        origin=Origin(xyz=(0.0, 0.02, 0.16)),
    )

    z_carriage = model.part("z_carriage")
    _add_mesh_visual(
        z_carriage,
        _make_z_carriage_body(),
        filename="z_carriage_body.obj",
        name="z_carriage_body",
        material=structure,
    )
    _add_mesh_visual(
        z_carriage,
        _make_z_bearing_pair(Z_BEARING_X),
        filename="z_left_bearings.obj",
        name="z_left_bearings",
        material=bearing_black,
    )
    _add_mesh_visual(
        z_carriage,
        _make_z_bearing_pair(-Z_BEARING_X),
        filename="z_right_bearings.obj",
        name="z_right_bearings",
        material=bearing_black,
    )
    z_carriage.visual(
        Box((TOOL_PLATE_WIDTH, TOOL_PLATE_DEPTH, TOOL_PLATE_HEIGHT)),
        origin=Origin(
            xyz=(
                0.0,
                TOOL_PLATE_BACK_Y + TOOL_PLATE_DEPTH / 2.0,
                0.0,
            )
        ),
        material=datum_metal,
        name="tool_plate",
    )
    z_carriage.inertial = Inertial.from_geometry(
        Box((0.14, 0.08, 0.16)),
        mass=4.8,
        origin=Origin(xyz=(0.0, 0.04, 0.0)),
    )

    model.articulation(
        "base_to_x",
        ArticulationType.PRISMATIC,
        parent=base,
        child=x_carriage,
        origin=Origin(xyz=(0.0, 0.0, X_RAIL_TOP)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=450.0,
            velocity=0.40,
            lower=-X_TRAVEL,
            upper=X_TRAVEL,
        ),
    )
    model.articulation(
        "x_to_y",
        ArticulationType.PRISMATIC,
        parent=x_carriage,
        child=y_carriage,
        origin=Origin(xyz=(0.0, 0.0, Y_RAIL_TOP)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=300.0,
            velocity=0.35,
            lower=-Y_TRAVEL,
            upper=Y_TRAVEL,
        ),
    )
    model.articulation(
        "y_to_z",
        ArticulationType.PRISMATIC,
        parent=y_carriage,
        child=z_carriage,
        origin=Origin(
            xyz=(
                0.0,
                Z_RAIL_FRONT_Y,
                Z_RAIL_BOTTOM + Z_RAIL_HEIGHT / 2.0,
            )
        ),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=220.0,
            velocity=0.25,
            lower=-Z_TRAVEL,
            upper=Z_TRAVEL,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    base = object_model.get_part("base_frame")
    x_carriage = object_model.get_part("x_carriage")
    y_carriage = object_model.get_part("y_carriage")
    z_carriage = object_model.get_part("z_carriage")

    x_slide = object_model.get_articulation("base_to_x")
    y_slide = object_model.get_articulation("x_to_y")
    z_slide = object_model.get_articulation("y_to_z")

    base_body = base.get_visual("base_body")
    x_left_rail = base.get_visual("x_left_rail")
    x_right_rail = base.get_visual("x_right_rail")
    x_left_guard = base.get_visual("x_left_guard")
    x_right_guard = base.get_visual("x_right_guard")
    base_left_cover = base.get_visual("base_left_cover")
    base_right_cover = base.get_visual("base_right_cover")
    base_datum_pad = base.get_visual("base_datum_pad")

    x_carriage_body = x_carriage.get_visual("x_carriage_body")
    x_left_bearings = x_carriage.get_visual("x_left_bearings")
    x_right_bearings = x_carriage.get_visual("x_right_bearings")
    y_left_rail = x_carriage.get_visual("y_left_rail")
    y_right_rail = x_carriage.get_visual("y_right_rail")
    y_left_guard = x_carriage.get_visual("y_left_guard")
    y_right_guard = x_carriage.get_visual("y_right_guard")
    y_left_guard_bracket = x_carriage.get_visual("y_left_guard_bracket")
    y_right_guard_bracket = x_carriage.get_visual("y_right_guard_bracket")
    x_access_cover = x_carriage.get_visual("x_access_cover")

    y_carriage_body = y_carriage.get_visual("y_carriage_body")
    y_left_bearings = y_carriage.get_visual("y_left_bearings")
    y_right_bearings = y_carriage.get_visual("y_right_bearings")
    z_left_rail = y_carriage.get_visual("z_left_rail")
    z_right_rail = y_carriage.get_visual("z_right_rail")
    z_left_guard = y_carriage.get_visual("z_left_guard")
    z_right_guard = y_carriage.get_visual("z_right_guard")
    z_access_cover = y_carriage.get_visual("z_access_cover")

    z_carriage_body = z_carriage.get_visual("z_carriage_body")
    z_left_bearings = z_carriage.get_visual("z_left_bearings")
    z_right_bearings = z_carriage.get_visual("z_right_bearings")
    tool_plate = z_carriage.get_visual("tool_plate")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Preferred default QC stack:
    # 1) likely-failure broad-part floating check for isolated parts
    ctx.fail_if_isolated_parts(max_pose_samples=7)
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()
    ctx.fail_if_parts_overlap_in_sampled_poses(max_pose_samples=21, ignore_adjacent=False, ignore_fixed=True)

    ctx.expect_gap(
        base,
        base,
        axis="y",
        positive_elem=x_left_guard,
        negative_elem=x_left_rail,
        min_gap=0.025,
        max_gap=0.060,
    )
    ctx.expect_gap(
        base,
        base,
        axis="y",
        positive_elem=x_right_rail,
        negative_elem=x_right_guard,
        min_gap=0.025,
        max_gap=0.060,
    )
    ctx.expect_overlap(base, base, axes="x", elem_a=x_left_guard, elem_b=x_left_rail, min_overlap=0.50)
    ctx.expect_within(base, base, axes="xy", inner_elem=base_left_cover, outer_elem=base_body, margin=0.0)
    ctx.expect_within(base, base, axes="xy", inner_elem=base_right_cover, outer_elem=base_body, margin=0.0)
    ctx.expect_within(base, base, axes="xy", inner_elem=base_datum_pad, outer_elem=base_body, margin=0.0)

    ctx.expect_contact(x_carriage, base, elem_a=x_left_bearings, elem_b=x_left_rail)
    ctx.expect_contact(x_carriage, base, elem_a=x_right_bearings, elem_b=x_right_rail)
    ctx.expect_gap(
        x_carriage,
        x_carriage,
        axis="x",
        positive_elem=y_left_guard,
        negative_elem=y_left_rail,
        min_gap=0.012,
        max_gap=0.040,
    )
    ctx.expect_gap(
        x_carriage,
        x_carriage,
        axis="x",
        positive_elem=y_right_rail,
        negative_elem=y_right_guard,
        min_gap=0.012,
        max_gap=0.040,
    )
    ctx.expect_contact(x_carriage, x_carriage, elem_a=y_left_guard_bracket, elem_b=x_carriage_body)
    ctx.expect_contact(x_carriage, x_carriage, elem_a=y_left_guard_bracket, elem_b=y_left_guard)
    ctx.expect_contact(x_carriage, x_carriage, elem_a=y_right_guard_bracket, elem_b=x_carriage_body)
    ctx.expect_contact(x_carriage, x_carriage, elem_a=y_right_guard_bracket, elem_b=y_right_guard)
    ctx.expect_within(x_carriage, x_carriage, axes="xy", inner_elem=x_access_cover, outer_elem=x_carriage_body, margin=0.0)

    ctx.expect_contact(y_carriage, x_carriage, elem_a=y_left_bearings, elem_b=y_left_rail)
    ctx.expect_contact(y_carriage, x_carriage, elem_a=y_right_bearings, elem_b=y_right_rail)
    ctx.expect_overlap(y_carriage, x_carriage, axes="xy", min_overlap=0.10)
    ctx.expect_gap(
        y_carriage,
        y_carriage,
        axis="x",
        positive_elem=z_left_guard,
        negative_elem=z_left_rail,
        min_gap=0.010,
        max_gap=0.035,
    )
    ctx.expect_gap(
        y_carriage,
        y_carriage,
        axis="x",
        positive_elem=z_right_rail,
        negative_elem=z_right_guard,
        min_gap=0.010,
        max_gap=0.035,
    )
    ctx.expect_within(y_carriage, y_carriage, axes="xz", inner_elem=z_access_cover, outer_elem=y_carriage_body, margin=0.0)

    ctx.expect_contact(z_carriage, y_carriage, elem_a=z_left_bearings, elem_b=z_left_rail)
    ctx.expect_contact(z_carriage, y_carriage, elem_a=z_right_bearings, elem_b=z_right_rail)
    ctx.expect_overlap(z_carriage, y_carriage, axes="xz", min_overlap=0.10)
    ctx.expect_within(z_carriage, z_carriage, axes="xz", inner_elem=tool_plate, outer_elem=z_carriage_body, margin=0.02)

    with ctx.pose({x_slide: X_TRAVEL, y_slide: -Y_TRAVEL, z_slide: Z_TRAVEL}):
        ctx.expect_contact(x_carriage, base, elem_a=x_left_bearings, elem_b=x_left_rail)
        ctx.expect_contact(x_carriage, base, elem_a=x_right_bearings, elem_b=x_right_rail)
        ctx.expect_contact(y_carriage, x_carriage, elem_a=y_left_bearings, elem_b=y_left_rail)
        ctx.expect_contact(y_carriage, x_carriage, elem_a=y_right_bearings, elem_b=y_right_rail)
        ctx.expect_contact(z_carriage, y_carriage, elem_a=z_left_bearings, elem_b=z_left_rail)
        ctx.expect_contact(z_carriage, y_carriage, elem_a=z_right_bearings, elem_b=z_right_rail)
        ctx.expect_overlap(z_carriage, y_carriage, axes="xz", min_overlap=0.10)

    with ctx.pose({x_slide: -X_TRAVEL, y_slide: Y_TRAVEL, z_slide: -Z_TRAVEL}):
        ctx.expect_contact(x_carriage, base, elem_a=x_left_bearings, elem_b=x_left_rail)
        ctx.expect_contact(y_carriage, x_carriage, elem_a=y_right_bearings, elem_b=y_right_rail)
        ctx.expect_contact(z_carriage, y_carriage, elem_a=z_right_bearings, elem_b=z_right_rail)

    _check_prismatic_motion(
        ctx,
        articulation=x_slide,
        child=x_carriage,
        axis="x",
        lower=-X_TRAVEL,
        upper=X_TRAVEL,
        name="x_slide_moves_only_along_x",
    )
    _check_prismatic_motion(
        ctx,
        articulation=y_slide,
        child=y_carriage,
        axis="y",
        lower=-Y_TRAVEL,
        upper=Y_TRAVEL,
        name="y_slide_moves_only_along_y",
    )
    _check_prismatic_motion(
        ctx,
        articulation=z_slide,
        child=z_carriage,
        axis="z",
        lower=-Z_TRAVEL,
        upper=Z_TRAVEL,
        name="z_slide_moves_only_along_z",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
