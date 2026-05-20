from __future__ import annotations

# The harness only exposes the editable block to the model.
# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    Inertial,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)

ASSETS = AssetContext.from_script(__file__)
HERE = ASSETS.asset_root

BODY_WIDTH = 0.76
BODY_DEPTH = 0.64
BODY_HEIGHT = 0.87
OPENING_FRONT_Y = 0.315
CONTROL_PANEL_FRONT_Y = 0.35
COOKTOP_TOP_Z = 0.90
DOOR_HINGE_Z = 0.12
KNOB_Z = 0.805

KNOB_LAYOUT = [
    ("knob_front_left", -0.28),
    ("knob_rear_left", -0.14),
    ("knob_center", 0.0),
    ("knob_rear_right", 0.14),
    ("knob_front_right", 0.28),
]


def _make_material(name: str, rgba: tuple[float, float, float, float]) -> Material:
    constructors = (
        lambda: Material(name=name, color=rgba),
        lambda: Material(name=name, rgba=rgba),
        lambda: Material(name, rgba),
    )
    for ctor in constructors:
        try:
            return ctor()
        except TypeError:
            continue
    raise TypeError(f"Unable to construct Material for {name!r}")


def _box(
    part,
    size: tuple[float, float, float],
    xyz: tuple[float, float, float],
    material: Material,
    *,
    rpy: tuple[float, float, float] = (0.0, 0.0, 0.0),
    name: str | None = None,
) -> None:
    part.visual(
        Box(size=size),
        origin=Origin(xyz=xyz, rpy=rpy),
        material=material,
        name=name,
    )


def _cylinder(
    part,
    radius: float,
    length: float,
    xyz: tuple[float, float, float],
    material: Material,
    *,
    rpy: tuple[float, float, float] = (0.0, 0.0, 0.0),
    name: str | None = None,
) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=rpy),
        material=material,
        name=name,
    )


def _add_knob(
    model: ArticulatedObject,
    name: str,
    x_pos: float,
    materials: dict[str, Material],
) -> None:
    knob = model.part(name)

    _cylinder(
        knob,
        radius=0.026,
        length=0.022,
        xyz=(0.0, 0.014, 0.0),
        rpy=(pi / 2.0, 0.0, 0.0),
        material=materials["knob_black"],
        name=f"{name}_skirt",
    )
    _cylinder(
        knob,
        radius=0.029,
        length=0.012,
        xyz=(0.0, 0.031, 0.0),
        rpy=(pi / 2.0, 0.0, 0.0),
        material=materials["knob_black"],
        name=f"{name}_face",
    )
    _cylinder(
        knob,
        radius=0.020,
        length=0.004,
        xyz=(0.0, 0.038, 0.0),
        rpy=(pi / 2.0, 0.0, 0.0),
        material=materials["dark_steel"],
        name=f"{name}_center_cap",
    )
    _box(
        knob,
        size=(0.005, 0.010, 0.018),
        xyz=(0.0, 0.036, 0.017),
        material=materials["chrome"],
        name=f"{name}_indicator",
    )
    _box(
        knob,
        size=(0.004, 0.012, 0.018),
        xyz=(-0.021, 0.020, 0.0),
        material=materials["knob_black"],
        name=f"{name}_left_grip",
    )
    _box(
        knob,
        size=(0.004, 0.012, 0.018),
        xyz=(0.021, 0.020, 0.0),
        material=materials["knob_black"],
        name=f"{name}_right_grip",
    )
    knob.inertial = Inertial.from_geometry(
        Cylinder(radius=0.03, length=0.04),
        mass=0.18,
        origin=Origin(xyz=(0.0, 0.020, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
    )

    model.articulation(
        f"{name}_joint",
        ArticulationType.REVOLUTE,
        parent="body",
        child=name,
        origin=Origin(xyz=(x_pos, CONTROL_PANEL_FRONT_Y, KNOB_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.5,
            velocity=4.0,
            lower=-2.4,
            upper=0.35,
        ),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="freestanding_range", assets=ASSETS)

    materials = {
        "stainless": _make_material("stainless", (0.74, 0.75, 0.77, 1.0)),
        "dark_steel": _make_material("dark_steel", (0.28, 0.29, 0.31, 1.0)),
        "black_enamel": _make_material("black_enamel", (0.07, 0.07, 0.08, 1.0)),
        "cast_iron": _make_material("cast_iron", (0.16, 0.16, 0.17, 1.0)),
        "knob_black": _make_material("knob_black", (0.05, 0.05, 0.06, 1.0)),
        "glass": _make_material("glass", (0.10, 0.12, 0.15, 0.28)),
        "chrome": _make_material("chrome", (0.84, 0.85, 0.87, 1.0)),
        "display_blue": _make_material("display_blue", (0.18, 0.36, 0.58, 0.60)),
    }
    model.materials.extend(list(materials.values()))

    body = model.part("body")
    _box(
        body,
        size=(0.025, BODY_DEPTH, BODY_HEIGHT),
        xyz=(-0.3675, 0.0, BODY_HEIGHT / 2.0),
        material=materials["stainless"],
        name="left_side_panel",
    )
    _box(
        body,
        size=(0.025, BODY_DEPTH, BODY_HEIGHT),
        xyz=(0.3675, 0.0, BODY_HEIGHT / 2.0),
        material=materials["stainless"],
        name="right_side_panel",
    )
    _box(
        body,
        size=(0.71, 0.02, 0.85),
        xyz=(0.0, -0.31, 0.425),
        material=materials["dark_steel"],
        name="rear_panel",
    )
    _box(
        body,
        size=(0.71, 0.60, 0.025),
        xyz=(0.0, -0.01, 0.0125),
        material=materials["dark_steel"],
        name="bottom_deck",
    )
    _box(
        body,
        size=(0.71, 0.60, 0.025),
        xyz=(0.0, -0.01, 0.78),
        material=materials["dark_steel"],
        name="oven_roof",
    )
    _box(
        body,
        size=(0.71, 0.05, 0.11),
        xyz=(0.0, 0.29, 0.055),
        material=materials["black_enamel"],
        name="toe_kick",
    )
    _box(
        body,
        size=(0.60, 0.022, 0.012),
        xyz=(0.0, 0.286, 0.106),
        material=materials["black_enamel"],
        name="door_threshold",
    )
    _box(
        body,
        size=(0.055, 0.045, 0.625),
        xyz=(-0.3275, 0.2925, 0.4225),
        material=materials["stainless"],
        name="left_front_stile",
    )
    _box(
        body,
        size=(0.055, 0.045, 0.625),
        xyz=(0.3275, 0.2925, 0.4225),
        material=materials["stainless"],
        name="right_front_stile",
    )
    _box(
        body,
        size=(0.71, 0.06, 0.09),
        xyz=(0.0, 0.285, 0.7775),
        material=materials["dark_steel"],
        name="upper_front_rail",
    )
    _box(
        body,
        size=(0.72, 0.045, 0.12),
        xyz=(0.0, 0.3275, 0.815),
        material=materials["stainless"],
        name="control_fascia",
    )
    _box(
        body,
        size=(0.14, 0.006, 0.045),
        xyz=(0.0, 0.35, 0.825),
        material=materials["display_blue"],
        name="display_glass",
    )
    _box(
        body,
        size=(0.56, 0.012, 0.015),
        xyz=(0.0, 0.314, 0.7425),
        material=materials["black_enamel"],
        name="oven_vent",
    )
    _box(
        body,
        size=(BODY_WIDTH, 0.67, 0.03),
        xyz=(0.0, 0.0, 0.885),
        material=materials["black_enamel"],
        name="cooktop_slab",
    )

    burner_specs = [
        (-0.19, 0.17, 0.085, 0.058, 0.040),
        (0.19, 0.17, 0.095, 0.066, 0.046),
        (-0.19, -0.14, 0.078, 0.054, 0.038),
        (0.19, -0.14, 0.082, 0.056, 0.039),
    ]
    for index, (x_pos, y_pos, bowl_r, ring_r, cap_r) in enumerate(burner_specs, start=1):
        _cylinder(
            body,
            radius=bowl_r,
            length=0.008,
            xyz=(x_pos, y_pos, 0.889),
            material=materials["dark_steel"],
            name=f"burner_bowl_{index}",
        )
        _cylinder(
            body,
            radius=ring_r,
            length=0.006,
            xyz=(x_pos, y_pos, 0.893),
            material=materials["cast_iron"],
            name=f"burner_ring_{index}",
        )
        _cylinder(
            body,
            radius=cap_r,
            length=0.020,
            xyz=(x_pos, y_pos, 0.901),
            material=materials["black_enamel"],
            name=f"burner_cap_{index}",
        )

    for side_name, x_center in (("left", -0.19), ("right", 0.19)):
        z_center = 0.9085
        _box(
            body,
            size=(0.31, 0.018, 0.018),
            xyz=(x_center, 0.25, z_center),
            material=materials["cast_iron"],
            name=f"{side_name}_grate_front",
        )
        _box(
            body,
            size=(0.31, 0.018, 0.018),
            xyz=(x_center, -0.21, z_center),
            material=materials["cast_iron"],
            name=f"{side_name}_grate_rear",
        )
        _box(
            body,
            size=(0.018, 0.46, 0.018),
            xyz=(x_center - 0.155, 0.02, z_center),
            material=materials["cast_iron"],
            name=f"{side_name}_grate_outer",
        )
        _box(
            body,
            size=(0.018, 0.46, 0.018),
            xyz=(x_center + 0.155, 0.02, z_center),
            material=materials["cast_iron"],
            name=f"{side_name}_grate_inner",
        )
        _box(
            body,
            size=(0.018, 0.36, 0.018),
            xyz=(x_center, 0.02, z_center),
            material=materials["cast_iron"],
            name=f"{side_name}_grate_center",
        )
        _box(
            body,
            size=(0.24, 0.018, 0.018),
            xyz=(x_center, 0.02, z_center),
            material=materials["cast_iron"],
            name=f"{side_name}_grate_cross",
        )

    body.inertial = Inertial.from_geometry(
        Box(size=(BODY_WIDTH, 0.67, 0.91)),
        mass=58.0,
        origin=Origin(xyz=(0.0, 0.0, 0.455)),
    )

    door = model.part("oven_door")
    _box(
        door,
        size=(0.585, 0.042, 0.59),
        xyz=(0.0, -0.021, 0.295),
        material=materials["stainless"],
        name="door_outer_panel",
    )
    _box(
        door,
        size=(0.42, 0.008, 0.34),
        xyz=(0.0, 0.002, 0.285),
        material=materials["glass"],
        name="door_glass",
    )
    _box(
        door,
        size=(0.47, 0.012, 0.03),
        xyz=(0.0, -0.002, 0.47),
        material=materials["dark_steel"],
        name="door_upper_trim",
    )
    _box(
        door,
        size=(0.022, 0.052, 0.024),
        xyz=(-0.22, 0.026, 0.545),
        material=materials["chrome"],
        name="handle_left_post",
    )
    _box(
        door,
        size=(0.022, 0.052, 0.024),
        xyz=(0.22, 0.026, 0.545),
        material=materials["chrome"],
        name="handle_right_post",
    )
    _cylinder(
        door,
        radius=0.012,
        length=0.46,
        xyz=(0.0, 0.054, 0.545),
        rpy=(0.0, pi / 2.0, 0.0),
        material=materials["chrome"],
        name="handle_bar",
    )
    door.inertial = Inertial.from_geometry(
        Box(size=(0.60, 0.08, 0.60)),
        mass=12.5,
        origin=Origin(xyz=(0.0, 0.0, 0.295)),
    )

    model.articulation(
        "oven_door_hinge",
        ArticulationType.REVOLUTE,
        parent="body",
        child="oven_door",
        origin=Origin(xyz=(0.0, OPENING_FRONT_Y, DOOR_HINGE_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=35.0,
            velocity=2.5,
            lower=0.0,
            upper=1.35,
        ),
    )

    for knob_name, knob_x in KNOB_LAYOUT:
        _add_knob(model, knob_name, knob_x, materials)

    return model


def _assert(condition: bool, message: str) -> None:
    if not condition:
        raise AssertionError(message)


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=HERE)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.01)
    ctx.fail_if_part_contains_disconnected_geometry_islands(use="visual")
    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=192,
        overlap_tol=0.004,
        overlap_volume_tol=0.0,
    )

    ctx.expect_aabb_overlap("oven_door", "body", axes="xy", min_overlap=0.025)
    with ctx.pose(oven_door_hinge=0.22):
        ctx.expect_aabb_overlap("oven_door", "body", axes="xy", min_overlap=0.03)
    ctx.expect_joint_motion_axis(
        "oven_door_hinge",
        "oven_door",
        world_axis="y",
        direction="positive",
        min_delta=0.20,
    )
    ctx.expect_joint_motion_axis(
        "oven_door_hinge",
        "oven_door",
        world_axis="z",
        direction="negative",
        min_delta=0.14,
    )

    _assert(
        len(object_model.parts) == 2 + len(KNOB_LAYOUT), "Unexpected part count for the range model"
    )
    _assert(
        len(object_model.articulations) == 1 + len(KNOB_LAYOUT),
        "Unexpected articulation count for the range model",
    )

    door_joint = object_model.get_articulation("oven_door_hinge")
    _assert(
        door_joint.articulation_type == ArticulationType.REVOLUTE, "Door must use a revolute joint"
    )
    _assert(
        tuple(door_joint.axis) == (-1.0, 0.0, 0.0),
        "Door hinge should rotate about the appliance width axis",
    )
    _assert(door_joint.motion_limits is not None, "Door hinge must define motion limits")
    _assert(door_joint.motion_limits.lower <= 0.0 + 1e-9, "Door hinge should close at zero radians")
    _assert(
        door_joint.motion_limits.upper >= 1.30, "Door hinge should open to a wide service angle"
    )

    knob_positions = [ctx.part_world_position(name) for name, _ in KNOB_LAYOUT]
    for (name, expected_x), position in zip(KNOB_LAYOUT, knob_positions):
        _assert(
            abs(position[0] - expected_x) < 1e-6,
            f"{name} origin drifted away from the control layout",
        )
        _assert(
            abs(position[1] - CONTROL_PANEL_FRONT_Y) < 1e-6,
            f"{name} should sit on the front control panel",
        )
        _assert(abs(position[2] - KNOB_Z) < 1e-6, f"{name} should stay on the control-height band")
    _assert(
        all(
            knob_positions[index][0] < knob_positions[index + 1][0]
            for index in range(len(knob_positions) - 1)
        ),
        "Knobs should progress left-to-right across the panel",
    )

    center_knob_rest = ctx.part_world_position("knob_center")
    with ctx.pose(knob_center_joint=-2.0):
        center_knob_rotated = ctx.part_world_position("knob_center")
    _assert(
        max(abs(a - b) for a, b in zip(center_knob_rest, center_knob_rotated)) < 1e-9,
        "Control knobs should spin in place instead of translating",
    )

    for knob_name, _ in KNOB_LAYOUT:
        joint = object_model.get_articulation(f"{knob_name}_joint")
        _assert(
            joint.articulation_type == ArticulationType.REVOLUTE,
            f"{knob_name} must be a revolute control",
        )
        _assert(
            tuple(joint.axis) == (0.0, 1.0, 0.0),
            f"{knob_name} should rotate about the front-facing axis",
        )
        _assert(joint.motion_limits is not None, f"{knob_name} should define usable control limits")
        _assert(
            joint.motion_limits.lower <= -2.2,
            f"{knob_name} should turn through a realistic cook-setting sweep",
        )
        _assert(
            joint.motion_limits.upper >= 0.25,
            f"{knob_name} should return slightly past the off detent",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
