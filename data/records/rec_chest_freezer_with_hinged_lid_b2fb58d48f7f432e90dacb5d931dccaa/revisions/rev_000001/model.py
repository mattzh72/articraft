from __future__ import annotations

from math import asin, cos, pi, sin, sqrt

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
)


OPEN_ANGLE = 1.10
LID_DEPTH = 0.84
LID_WIDTH = 1.50
LID_THICKNESS = 0.11
HINGE_X = -0.43
HINGE_Z = 0.865


def _open_lid_xyz(local_xyz: tuple[float, float, float]) -> tuple[float, float, float]:
    """Map a point in the lid's closed, hinge-local coordinates into q=0 world."""
    x, y, z = _open_lid_rel(local_xyz)
    return (HINGE_X + x, y, HINGE_Z + z)


def _open_lid_rel(local_xyz: tuple[float, float, float]) -> tuple[float, float, float]:
    """Map a point in the lid's closed coordinates into the lid child frame."""
    x, y, z = local_xyz
    c = cos(OPEN_ANGLE)
    s = sin(OPEN_ANGLE)
    # The lid shell is pre-rotated by -OPEN_ANGLE so q=0 is the held-open pose.
    return (c * x - s * z, y, s * x + c * z)


def _lid_origin(local_xyz: tuple[float, float, float]) -> Origin:
    return Origin(xyz=_open_lid_rel(local_xyz), rpy=(0.0, -OPEN_ANGLE, 0.0))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="commercial_chest_freezer")

    stainless = model.material("brushed_stainless", rgba=(0.68, 0.70, 0.69, 1.0))
    dark_stainless = model.material("dark_stainless", rgba=(0.42, 0.44, 0.44, 1.0))
    liner = model.material("white_liner", rgba=(0.93, 0.96, 0.98, 1.0))
    gasket = model.material("black_rubber", rgba=(0.01, 0.012, 0.011, 1.0))
    black = model.material("black_plastic", rgba=(0.02, 0.02, 0.022, 1.0))
    chrome = model.material("polished_chrome", rgba=(0.82, 0.84, 0.83, 1.0))
    label_blue = model.material("blue_label", rgba=(0.04, 0.13, 0.35, 1.0))
    marker = model.material("white_marker", rgba=(1.0, 1.0, 0.94, 1.0))

    cabinet = model.part("cabinet")
    # Hollow stainless outer tub with an open top.
    cabinet.visual(
        Box((0.05, 1.45, 0.74)),
        origin=Origin(xyz=(0.365, 0.0, 0.45)),
        material=stainless,
        name="front_wall",
    )
    cabinet.visual(
        Box((0.05, 1.45, 0.74)),
        origin=Origin(xyz=(-0.365, 0.0, 0.45)),
        material=stainless,
        name="rear_wall",
    )
    cabinet.visual(
        Box((0.78, 0.05, 0.74)),
        origin=Origin(xyz=(0.0, 0.70, 0.45)),
        material=stainless,
        name="side_wall_0",
    )
    cabinet.visual(
        Box((0.78, 0.05, 0.74)),
        origin=Origin(xyz=(0.0, -0.70, 0.45)),
        material=stainless,
        name="side_wall_1",
    )
    cabinet.visual(
        Box((0.78, 1.45, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 0.08)),
        material=dark_stainless,
        name="bottom_pan",
    )
    # Light-colored inner liner, lower than the rim so the chest reads hollow.
    cabinet.visual(
        Box((0.66, 1.30, 0.045)),
        origin=Origin(xyz=(0.0, 0.0, 0.145)),
        material=liner,
        name="liner_floor",
    )
    cabinet.visual(
        Box((0.040, 1.30, 0.55)),
        origin=Origin(xyz=(0.330, 0.0, 0.43)),
        material=liner,
        name="front_liner",
    )
    cabinet.visual(
        Box((0.040, 1.30, 0.55)),
        origin=Origin(xyz=(-0.330, 0.0, 0.43)),
        material=liner,
        name="rear_liner",
    )
    cabinet.visual(
        Box((0.66, 0.050, 0.55)),
        origin=Origin(xyz=(0.0, 0.655, 0.43)),
        material=liner,
        name="side_liner_0",
    )
    cabinet.visual(
        Box((0.66, 0.050, 0.55)),
        origin=Origin(xyz=(0.0, -0.655, 0.43)),
        material=liner,
        name="side_liner_1",
    )
    # Heavy upper rim and continuous rubber gasket.
    cabinet.visual(
        Box((0.09, 1.46, 0.045)),
        origin=Origin(xyz=(0.365, 0.0, 0.8225)),
        material=stainless,
        name="front_rim",
    )
    cabinet.visual(
        Box((0.09, 1.46, 0.045)),
        origin=Origin(xyz=(-0.365, 0.0, 0.8225)),
        material=stainless,
        name="rear_rim",
    )
    cabinet.visual(
        Box((0.78, 0.09, 0.045)),
        origin=Origin(xyz=(0.0, 0.685, 0.8225)),
        material=stainless,
        name="side_rim_0",
    )
    cabinet.visual(
        Box((0.78, 0.09, 0.045)),
        origin=Origin(xyz=(0.0, -0.685, 0.8225)),
        material=stainless,
        name="side_rim_1",
    )
    cabinet.visual(
        Box((0.055, 1.34, 0.020)),
        origin=Origin(xyz=(0.337, 0.0, 0.855)),
        material=gasket,
        name="front_gasket",
    )
    cabinet.visual(
        Box((0.055, 1.34, 0.020)),
        origin=Origin(xyz=(-0.337, 0.0, 0.855)),
        material=gasket,
        name="rear_gasket",
    )
    cabinet.visual(
        Box((0.62, 0.055, 0.020)),
        origin=Origin(xyz=(0.0, 0.642, 0.855)),
        material=gasket,
        name="side_gasket_0",
    )
    cabinet.visual(
        Box((0.62, 0.055, 0.020)),
        origin=Origin(xyz=(0.0, -0.642, 0.855)),
        material=gasket,
        name="side_gasket_1",
    )
    # Full-width piano hinge details along the rear edge.
    cabinet.visual(
        Box((0.006, 1.46, 0.060)),
        origin=Origin(xyz=(-0.393, 0.0, 0.820)),
        material=stainless,
        name="rear_hinge_leaf",
    )
    cabinet.visual(
        Cylinder(radius=0.012, length=1.50),
        origin=Origin(xyz=(-0.406, 0.0, 0.842), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_stainless,
        name="hinge_pin",
    )
    for i, y in enumerate((-0.60, -0.36, -0.12, 0.12, 0.36, 0.60)):
        cabinet.visual(
            Cylinder(radius=0.018, length=0.105),
            origin=Origin(xyz=(-0.406, y, 0.842), rpy=(pi / 2.0, 0.0, 0.0)),
            material=stainless,
            name=f"hinge_knuckle_{i}",
        )
    # Commercial details: vented compressor grille, data label, leveling feet.
    cabinet.visual(
        Box((0.012, 0.42, 0.18)),
        origin=Origin(xyz=(0.393, 0.36, 0.33)),
        material=black,
        name="vent_shadow",
    )
    for i, z in enumerate((0.27, 0.31, 0.35, 0.39)):
        cabinet.visual(
            Box((0.014, 0.38, 0.012)),
            origin=Origin(xyz=(0.401, 0.36, z)),
            material=dark_stainless,
            name=f"vent_louver_{i}",
        )
    cabinet.visual(
        Box((0.010, 0.17, 0.09)),
        origin=Origin(xyz=(0.395, -0.23, 0.39)),
        material=label_blue,
        name="data_label",
    )
    for i, (x, y) in enumerate(
        ((0.31, 0.58), (0.31, -0.58), (-0.31, 0.58), (-0.31, -0.58))
    ):
        cabinet.visual(
            Cylinder(radius=0.045, length=0.065),
            origin=Origin(xyz=(x, y, 0.0325)),
            material=black,
            name=f"foot_{i}",
        )
    # Brackets for the gas strut are fixed to the cabinet and lid respectively.
    cabinet.visual(
        Box((0.075, 0.028, 0.070)),
        origin=Origin(xyz=(0.20, 0.616, 0.54)),
        material=dark_stainless,
        name="strut_mount",
    )

    lid = model.part("lid")
    lid.visual(
        Box((LID_DEPTH, LID_WIDTH, LID_THICKNESS)),
        origin=_lid_origin((LID_DEPTH / 2.0, 0.0, LID_THICKNESS / 2.0)),
        material=stainless,
        name="lid_shell",
    )
    lid.visual(
        Box((0.70, 1.25, 0.030)),
        origin=_lid_origin((0.43, 0.0, -0.015)),
        material=liner,
        name="underside_liner",
    )
    lid.visual(
        Box((0.070, 1.30, 0.025)),
        origin=_lid_origin((0.80, 0.0, 0.045)),
        material=black,
        name="front_handle",
    )
    lid.visual(
        Box((0.055, 1.42, 0.006)),
        origin=_lid_origin((0.025, 0.0, 0.003)),
        material=stainless,
        name="hinge_leaf",
    )
    lid.visual(
        Box((0.090, 0.045, 0.110)),
        origin=_lid_origin((0.52, 0.58, -0.055)),
        material=dark_stainless,
        name="strut_socket",
    )

    model.articulation(
        "cabinet_to_lid",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=lid,
        origin=Origin(xyz=(HINGE_X, 0.0, HINGE_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=220.0, velocity=0.8, lower=-OPEN_ANGLE, upper=0.15
        ),
    )

    # Gas strut drawn in the held-open pose.  It has a pivoting cylinder body and
    # a telescoping polished rod, with the rod end seated in the lid socket.
    lower = (0.20, 0.58, 0.54)
    upper = _open_lid_xyz((0.52, 0.58, -0.095))
    dx = upper[0] - lower[0]
    dz = upper[2] - lower[2]
    strut_length = sqrt(dx * dx + dz * dz)
    strut_pitch = asin(dx / strut_length)
    tube_length = 0.43
    rod_length = strut_length - tube_length

    gas_tube = model.part("gas_tube")
    gas_tube.visual(
        Cylinder(radius=0.019, length=tube_length),
        origin=Origin(xyz=(0.0, 0.0, tube_length / 2.0)),
        material=black,
        name="strut_cylinder",
    )
    gas_tube.visual(
        Sphere(radius=0.024),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=dark_stainless,
        name="lower_eye",
    )
    gas_tube.visual(
        Cylinder(radius=0.023, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, tube_length - 0.015)),
        material=dark_stainless,
        name="end_collar",
    )
    model.articulation(
        "cabinet_to_gas_tube",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=gas_tube,
        origin=Origin(xyz=lower, rpy=(0.0, strut_pitch, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=30.0, velocity=1.2, lower=-0.35, upper=0.35),
    )

    gas_rod = model.part("gas_rod")
    gas_rod.visual(
        Cylinder(radius=0.008, length=rod_length),
        origin=Origin(xyz=(0.0, 0.0, rod_length / 2.0)),
        material=chrome,
        name="polished_rod",
    )
    gas_rod.visual(
        Sphere(radius=0.022),
        origin=Origin(xyz=(0.0, 0.0, rod_length)),
        material=dark_stainless,
        name="upper_eye",
    )
    model.articulation(
        "gas_tube_to_gas_rod",
        ArticulationType.PRISMATIC,
        parent=gas_tube,
        child=gas_rod,
        origin=Origin(xyz=(0.0, 0.0, tube_length)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=60.0, velocity=0.25, lower=0.0, upper=0.12),
    )

    thermostat_dial = model.part("thermostat_dial")
    thermostat_dial.visual(
        Cylinder(radius=0.038, length=0.030),
        origin=Origin(xyz=(0.015, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=black,
        name="dial_cap",
    )
    thermostat_dial.visual(
        Box((0.006, 0.010, 0.034)),
        origin=Origin(xyz=(0.033, 0.0, 0.017)),
        material=marker,
        name="dial_pointer",
    )
    model.articulation(
        "cabinet_to_thermostat_dial",
        ArticulationType.CONTINUOUS,
        parent=cabinet,
        child=thermostat_dial,
        origin=Origin(xyz=(0.390, -0.48, 0.36)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.4, velocity=3.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    cabinet = object_model.get_part("cabinet")
    lid = object_model.get_part("lid")
    gas_rod = object_model.get_part("gas_rod")
    lid_hinge = object_model.get_articulation("cabinet_to_lid")

    ctx.allow_overlap(
        lid,
        gas_rod,
        elem_a="strut_socket",
        elem_b="upper_eye",
        reason=(
            "The gas-strut ball end is intentionally captured inside the lid "
            "socket in the held-open pose."
        ),
    )
    ctx.allow_overlap(
        lid,
        gas_rod,
        elem_a="strut_socket",
        elem_b="polished_rod",
        reason=(
            "The last short section of the gas-strut rod passes into the clevis "
            "slot ahead of the captured ball end."
        ),
    )
    ctx.allow_overlap(
        cabinet,
        object_model.get_part("gas_tube"),
        elem_a="strut_mount",
        elem_b="lower_eye",
        reason=(
            "The lower gas-strut ball end is intentionally seated in the side "
            "mount clevis."
        ),
    )
    ctx.expect_overlap(
        lid,
        gas_rod,
        axes="xyz",
        min_overlap=0.004,
        elem_a="strut_socket",
        elem_b="upper_eye",
        name="gas strut end is seated in lid socket",
    )
    ctx.expect_overlap(
        lid,
        gas_rod,
        axes="yz",
        min_overlap=0.004,
        elem_a="strut_socket",
        elem_b="polished_rod",
        name="gas strut rod enters the lid clevis slot",
    )
    ctx.expect_overlap(
        cabinet,
        object_model.get_part("gas_tube"),
        axes="yz",
        min_overlap=0.002,
        elem_a="strut_mount",
        elem_b="lower_eye",
        name="lower gas strut ball is seated in cabinet mount",
    )

    with ctx.pose({lid_hinge: 0.0}):
        ctx.expect_gap(
            lid,
            cabinet,
            axis="z",
            positive_elem="lid_shell",
            negative_elem="front_gasket",
            min_gap=0.0,
            name="held-open lid clears the cabinet gasket",
        )
        open_aabb = ctx.part_world_aabb(lid)

    with ctx.pose({lid_hinge: -OPEN_ANGLE}):
        ctx.expect_gap(
            lid,
            cabinet,
            axis="z",
            positive_elem="lid_shell",
            negative_elem="front_gasket",
            max_gap=0.006,
            max_penetration=0.001,
            name="closed insulated lid lands on the gasket",
        )
        ctx.expect_overlap(
            lid,
            cabinet,
            axes="xy",
            min_overlap=0.70,
            elem_a="lid_shell",
            name="closed lid spans the chest opening",
        )
        closed_aabb = ctx.part_world_aabb(lid)

    ctx.check(
        "lid default pose is visibly held open",
        open_aabb is not None
        and closed_aabb is not None
        and open_aabb[1][2] > closed_aabb[1][2] + 0.30,
        details=f"open_aabb={open_aabb}, closed_aabb={closed_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
