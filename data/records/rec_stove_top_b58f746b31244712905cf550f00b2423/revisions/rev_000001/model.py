from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobBore,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    KnobSkirt,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


COUNTER_FRONT_Y = -0.370
CONTROL_LIP_FRONT_Y = -0.396
COOKTOP_TOP_Z = 0.877
DOOR_BOTTOM_Z = 0.100
DOOR_HEIGHT = 0.620
DOOR_WIDTH = 0.560


def _mat(name: str, rgba: tuple[float, float, float, float]) -> Material:
    return Material(name, rgba=rgba)


COUNTER = _mat("warm_speckled_countertop", (0.63, 0.58, 0.50, 1.0))
CABINET_WOOD = _mat("satin_walnut_cabinet", (0.46, 0.26, 0.12, 1.0))
DARK_RECESS = _mat("shadow_black_recess", (0.025, 0.022, 0.020, 1.0))
BLACK_GLASS = _mat("black_glass_cooktop", (0.015, 0.016, 0.018, 1.0))
CAST_IRON = _mat("matte_cast_iron", (0.018, 0.018, 0.017, 1.0))
BRUSHED_STEEL = _mat("brushed_stainless_steel", (0.70, 0.68, 0.62, 1.0))
KNOB_FACE = _mat("dark_graphite_knob", (0.09, 0.085, 0.080, 1.0))
INDICATOR_WHITE = _mat("white_ceramic_marker", (0.92, 0.88, 0.78, 1.0))


def _add_cabinet_and_cooktop(cabinet) -> None:
    """Static continuous cabinet/counter/cooktop assembly."""
    # Simple cabinet carcass under a broad countertop.
    cabinet.visual(
        Box((1.250, 0.580, 0.035)),
        origin=Origin(xyz=(0.0, 0.020, 0.0175)),
        material=CABINET_WOOD,
        name="bottom_deck",
    )
    cabinet.visual(
        Box((0.036, 0.584, 0.820)),
        origin=Origin(xyz=(-0.607, 0.020, 0.410)),
        material=CABINET_WOOD,
        name="side_stile_0",
    )
    cabinet.visual(
        Box((0.036, 0.584, 0.820)),
        origin=Origin(xyz=(0.607, 0.020, 0.410)),
        material=CABINET_WOOD,
        name="side_stile_1",
    )
    cabinet.visual(
        Box((1.250, 0.036, 0.820)),
        origin=Origin(xyz=(0.0, 0.312, 0.410)),
        material=CABINET_WOOD,
        name="rear_panel",
    )
    cabinet.visual(
        Box((1.245, 0.070, 0.070)),
        origin=Origin(xyz=(0.0, -0.292, 0.785)),
        material=CABINET_WOOD,
        name="front_top_rail",
    )
    cabinet.visual(
        Box((1.130, 0.070, 0.085)),
        origin=Origin(xyz=(0.0, -0.292, 0.062)),
        material=DARK_RECESS,
        name="toe_kick",
    )
    cabinet.visual(
        Box((0.026, 0.055, 0.660)),
        origin=Origin(xyz=(0.0, -0.314, 0.390)),
        material=CABINET_WOOD,
        name="center_mullion",
    )
    cabinet.visual(
        Box((1.360, 0.740, 0.045)),
        origin=Origin(xyz=(0.0, 0.0, 0.8425)),
        material=COUNTER,
        name="countertop",
    )
    cabinet.visual(
        Box((1.220, 0.026, 0.120)),
        origin=Origin(xyz=(0.0, -0.383, 0.782)),
        material=BRUSHED_STEEL,
        name="front_control_lip",
    )
    for i, x in enumerate((-0.590, 0.590)):
        cabinet.visual(
            Box((0.010, 0.026, DOOR_HEIGHT + 0.004)),
            origin=Origin(xyz=(x, -0.383, DOOR_BOTTOM_Z + (DOOR_HEIGHT + 0.004) / 2.0)),
            material=CABINET_WOOD,
            name=f"door_stop_{i}",
        )

    # Flush black-glass gas cooktop insert seated in the countertop.
    cabinet.visual(
        Box((1.030, 0.535, 0.014)),
        origin=Origin(xyz=(0.030, 0.030, 0.870)),
        material=BLACK_GLASS,
        name="cooktop_plate",
    )
    cabinet.visual(
        Box((1.060, 0.015, 0.010)),
        origin=Origin(xyz=(0.030, -0.245, 0.879)),
        material=BRUSHED_STEEL,
        name="cooktop_trim_front",
    )
    cabinet.visual(
        Box((1.060, 0.015, 0.010)),
        origin=Origin(xyz=(0.030, 0.305, 0.879)),
        material=BRUSHED_STEEL,
        name="cooktop_trim_rear",
    )
    cabinet.visual(
        Box((0.015, 0.565, 0.010)),
        origin=Origin(xyz=(-0.493, 0.030, 0.879)),
        material=BRUSHED_STEEL,
        name="cooktop_trim_0",
    )
    cabinet.visual(
        Box((0.015, 0.565, 0.010)),
        origin=Origin(xyz=(0.553, 0.030, 0.879)),
        material=BRUSHED_STEEL,
        name="cooktop_trim_1",
    )

    # Cast-iron grate bars, supported on small feet that sink just into the
    # glass plate so the static cooktop reads as one mounted assembly.
    grate_z = COOKTOP_TOP_Z + 0.029
    for i, y in enumerate((-0.185, -0.015, 0.155, 0.290)):
        cabinet.visual(
            Box((0.920, 0.020, 0.022)),
            origin=Origin(xyz=(0.035, y, grate_z)),
            material=CAST_IRON,
            name=f"grate_crossbar_{i}",
        )
    for i, x in enumerate((-0.395, -0.165, 0.160, 0.395)):
        cabinet.visual(
            Box((0.020, 0.535, 0.022)),
            origin=Origin(xyz=(x, 0.050, grate_z)),
            material=CAST_IRON,
            name=f"grate_rail_{i}",
        )
    for i, (x, y) in enumerate(
        (
            (-0.430, -0.205),
            (-0.430, 0.305),
            (0.455, -0.205),
            (0.455, 0.305),
            (-0.080, -0.205),
            (0.100, 0.305),
        )
    ):
        cabinet.visual(
            Box((0.036, 0.036, 0.020)),
            origin=Origin(xyz=(x, y, COOKTOP_TOP_Z + 0.009)),
            material=CAST_IRON,
            name=f"grate_foot_{i}",
        )

    # Five gas burners: four corner burners plus a larger central wok burner.
    burner_specs = (
        (-0.315, 0.205, 0.054, "rear_left"),
        (0.350, 0.205, 0.050, "rear_right"),
        (-0.315, -0.135, 0.047, "front_left"),
        (0.330, -0.135, 0.044, "front_right"),
        (0.030, 0.040, 0.068, "center"),
    )
    for x, y, ring_radius, tag in burner_specs:
        ring_mesh = mesh_from_geometry(
            TorusGeometry(ring_radius, 0.006, radial_segments=18, tubular_segments=48),
            f"{tag}_burner_ring",
        )
        cabinet.visual(
            ring_mesh,
            origin=Origin(xyz=(x, y, COOKTOP_TOP_Z + 0.005)),
            material=BRUSHED_STEEL,
            name=f"{tag}_burner_ring",
        )
        cabinet.visual(
            Cylinder(radius=ring_radius * 0.48, length=0.010),
            origin=Origin(xyz=(x, y, COOKTOP_TOP_Z + 0.004)),
            material=CAST_IRON,
            name=f"{tag}_burner_cap",
        )
        cabinet.visual(
            Cylinder(radius=0.006, length=0.016),
            origin=Origin(xyz=(x + ring_radius * 0.78, y - ring_radius * 0.20, COOKTOP_TOP_Z + 0.007)),
            material=INDICATOR_WHITE,
            name=f"{tag}_igniter",
        )


def _make_knob_meshes():
    large = KnobGeometry(
        0.048,
        0.030,
        body_style="skirted",
        top_diameter=0.038,
        skirt=KnobSkirt(0.056, 0.006, flare=0.08, chamfer=0.0012),
        grip=KnobGrip(style="fluted", count=22, depth=0.0013),
        indicator=KnobIndicator(style="line", mode="engraved", depth=0.0007),
        bore=KnobBore(style="d_shaft", diameter=0.0065, flat_depth=0.001),
        center=False,
    )
    small = KnobGeometry(
        0.040,
        0.026,
        body_style="skirted",
        top_diameter=0.031,
        skirt=KnobSkirt(0.047, 0.0045, flare=0.07, chamfer=0.001),
        grip=KnobGrip(style="fluted", count=18, depth=0.0010),
        indicator=KnobIndicator(style="line", mode="engraved", depth=0.0006),
        bore=KnobBore(style="d_shaft", diameter=0.0055, flat_depth=0.0008),
        center=False,
    )
    return (
        mesh_from_geometry(large, "large_range_knob"),
        mesh_from_geometry(small, "small_range_knob"),
    )


def _add_knob(
    model: ArticulatedObject,
    parent,
    name: str,
    mesh,
    x: float,
    z: float,
) -> None:
    knob = model.part(name)
    knob.visual(mesh, material=KNOB_FACE, name="knob_body")
    # The joint frame is rotated so the knob's local +Z axis points out of the
    # front face along world -Y.  Continuous rotation is therefore front-to-back.
    model.articulation(
        f"cabinet_to_{name}",
        ArticulationType.CONTINUOUS,
        parent=parent,
        child=knob,
        origin=Origin(xyz=(x, CONTROL_LIP_FRONT_Y, z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.5, velocity=8.0),
    )


def _add_door(model: ArticulatedObject, parent, index: int, hinge_x: float, direction: float) -> None:
    """Add one lower cabinet door.  direction is +1 for panel extending +X."""
    door = model.part(f"door_{index}")
    panel_center_x = direction * (DOOR_WIDTH / 2.0)
    inner_handle_x = direction * (DOOR_WIDTH - 0.060)
    outer_stile_x = direction * 0.040
    inner_stile_x = direction * (DOOR_WIDTH - 0.040)

    door.visual(
        Box((DOOR_WIDTH, 0.026, DOOR_HEIGHT)),
        origin=Origin(xyz=(panel_center_x, -0.013, DOOR_HEIGHT / 2.0)),
        material=CABINET_WOOD,
        name="door_panel",
    )
    for rail_name, z in (("top_rail", DOOR_HEIGHT - 0.050), ("bottom_rail", 0.050)):
        door.visual(
            Box((DOOR_WIDTH - 0.055, 0.008, 0.055)),
            origin=Origin(xyz=(panel_center_x, -0.030, z)),
            material=CABINET_WOOD,
            name=rail_name,
        )
    for stile_name, x in (("outer_stile", outer_stile_x), ("inner_stile", inner_stile_x)):
        door.visual(
            Box((0.055, 0.008, DOOR_HEIGHT - 0.075)),
            origin=Origin(xyz=(x, -0.030, DOOR_HEIGHT / 2.0)),
            material=CABINET_WOOD,
            name=stile_name,
        )
    # Slim vertical pull mounted by two bosses to avoid a floating handle.
    door.visual(
        Cylinder(radius=0.006, length=0.220),
        origin=Origin(xyz=(inner_handle_x, -0.059, DOOR_HEIGHT / 2.0)),
        material=BRUSHED_STEEL,
        name="pull_bar",
    )
    for i, z in enumerate((DOOR_HEIGHT / 2.0 - 0.075, DOOR_HEIGHT / 2.0 + 0.075)):
        door.visual(
            Box((0.024, 0.034, 0.020)),
            origin=Origin(xyz=(inner_handle_x, -0.040, z)),
            material=BRUSHED_STEEL,
            name=f"pull_boss_{i}",
        )

    axis_z = -1.0 if direction > 0.0 else 1.0
    model.articulation(
        f"cabinet_to_door_{index}",
        ArticulationType.REVOLUTE,
        parent=parent,
        child=door,
        origin=Origin(xyz=(hinge_x, -0.382, DOOR_BOTTOM_Z)),
        axis=(0.0, 0.0, axis_z),
        motion_limits=MotionLimits(effort=18.0, velocity=1.5, lower=0.0, upper=1.65),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="five_burner_gas_cooktop_cabinet")

    cabinet = model.part("cabinet")
    _add_cabinet_and_cooktop(cabinet)

    large_knob_mesh, small_knob_mesh = _make_knob_meshes()
    _add_knob(model, cabinet, "large_knob_0", large_knob_mesh, -0.505, 0.812)
    _add_knob(model, cabinet, "large_knob_1", large_knob_mesh, -0.505, 0.752)
    for i, x in enumerate((0.155, 0.340, 0.525)):
        _add_knob(model, cabinet, f"small_knob_{i}", small_knob_mesh, x, 0.780)

    _add_door(model, cabinet, index=0, hinge_x=-0.585, direction=1.0)
    _add_door(model, cabinet, index=1, hinge_x=0.585, direction=-1.0)
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    cabinet = object_model.get_part("cabinet")
    knob_names = [f"large_knob_{i}" for i in range(2)] + [f"small_knob_{i}" for i in range(3)]
    door_names = ["door_0", "door_1"]

    for name in knob_names:
        joint = object_model.get_articulation(f"cabinet_to_{name}")
        ctx.check(
            f"{name} is a continuous front-axis control",
            joint.articulation_type == ArticulationType.CONTINUOUS
            and tuple(round(v, 6) for v in joint.axis) == (0.0, 0.0, 1.0)
            and abs(joint.origin.rpy[0] - math.pi / 2.0) < 1.0e-6,
            details=f"type={joint.articulation_type}, axis={joint.axis}, rpy={joint.origin.rpy}",
        )
        ctx.expect_contact(
            name,
            cabinet,
            elem_a="knob_body",
            elem_b="front_control_lip",
            contact_tol=0.002,
            name=f"{name} is seated on front lip",
        )

    # Large controls form a vertical stack at the left; smaller controls form a
    # single row across the right half of the front lip.
    large_aabb_0 = ctx.part_world_aabb("large_knob_0")
    large_aabb_1 = ctx.part_world_aabb("large_knob_1")
    small_aabbs = [ctx.part_world_aabb(name) for name in ("small_knob_0", "small_knob_1", "small_knob_2")]
    ctx.check(
        "large knobs are vertically stacked",
        large_aabb_0 is not None
        and large_aabb_1 is not None
        and abs(((large_aabb_0[0][0] + large_aabb_0[1][0]) * 0.5) - ((large_aabb_1[0][0] + large_aabb_1[1][0]) * 0.5)) < 0.015
        and ((large_aabb_0[0][2] + large_aabb_0[1][2]) * 0.5) > ((large_aabb_1[0][2] + large_aabb_1[1][2]) * 0.5) + 0.035,
        details=f"large0={large_aabb_0}, large1={large_aabb_1}",
    )
    ctx.check(
        "small knobs run across right half",
        all(aabb is not None for aabb in small_aabbs)
        and all(((aabb[0][0] + aabb[1][0]) * 0.5) > 0.05 for aabb in small_aabbs if aabb is not None)
        and all(abs(((aabb[0][2] + aabb[1][2]) * 0.5) - 0.780) < 0.030 for aabb in small_aabbs if aabb is not None),
        details=f"small={small_aabbs}",
    )

    for name in door_names:
        hinge = object_model.get_articulation(f"cabinet_to_{name}")
        ctx.check(
            f"{name} has vertical hinge limits",
            hinge.articulation_type == ArticulationType.REVOLUTE
            and abs(hinge.axis[2]) == 1.0
            and hinge.motion_limits is not None
            and hinge.motion_limits.lower == 0.0
            and hinge.motion_limits.upper > 1.4,
            details=f"type={hinge.articulation_type}, axis={hinge.axis}, limits={hinge.motion_limits}",
        )
        closed = ctx.part_world_aabb(name)
        with ctx.pose({hinge: 1.10}):
            opened = ctx.part_world_aabb(name)
        ctx.check(
            f"{name} swings outward",
            closed is not None and opened is not None and opened[0][1] < closed[0][1] - 0.18,
            details=f"closed={closed}, opened={opened}",
        )

    return ctx.report()


object_model = build_object_model()
