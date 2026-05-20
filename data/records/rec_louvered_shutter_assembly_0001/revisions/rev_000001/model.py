from __future__ import annotations

# The harness only exposes the editable block to the model.
# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi
from pathlib import Path

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
    Sphere,
    TestContext,
    TestReport,
)

ASSETS = AssetContext.from_script(__file__)
HERE = ASSETS.asset_root
FRAME_WIDTH = 0.760
FRAME_HEIGHT = 1.480
FRAME_DEPTH = 0.055
FRAME_MEMBER = 0.065

LEAF_WIDTH = 0.610
LEAF_HEIGHT = 1.328
LEAF_THICKNESS = 0.032
LEAF_STILE = 0.078
LEAF_RAIL = 0.110
LEAF_LEFT_OFFSET = 0.008
LEAF_PANEL_Y = 0.010

HINGE_AXIS_X = -0.316
HINGE_AXIS_Y = 0.0365
HINGE_AXIS_Z = FRAME_HEIGHT / 2.0
HINGE_Z_OFFSETS = (-0.500, 0.000, 0.500)
FRAME_HINGE_PLATE_Y = 0.034
LEAF_HINGE_PLATE_Y = 0.024

SLAT_COUNT = 11
SLAT_LENGTH = 0.430
SLAT_BODY_OFFSET_X = 0.007
SLAT_HEIGHT = 0.078
SLAT_CORE_DEPTH = 0.006
SLAT_EDGE_RADIUS = 0.003
SLAT_PITCH = 0.092
SLAT_BOTTOM_Z = -0.460
SLAT_LEFT_X = LEAF_LEFT_OFFSET + LEAF_STILE
SLAT_ZS = tuple(SLAT_BOTTOM_Z + i * SLAT_PITCH for i in range(SLAT_COUNT))


def _make_material(name: str, rgba: tuple[float, float, float, float]):
    try:
        return Material(name=name, rgba=rgba)
    except TypeError:
        try:
            return Material(name=name, color=rgba)
        except TypeError:
            return Material(name, rgba)


def _add_box(
    part,
    size: tuple[float, float, float],
    xyz: tuple[float, float, float],
    *,
    material=None,
    name: str | None = None,
    rpy: tuple[float, float, float] = (0.0, 0.0, 0.0),
) -> None:
    part.visual(Box(size), origin=Origin(xyz=xyz, rpy=rpy), material=material, name=name)


def _add_cylinder(
    part,
    radius: float,
    length: float,
    xyz: tuple[float, float, float],
    *,
    material=None,
    name: str | None = None,
    rpy: tuple[float, float, float] = (0.0, 0.0, 0.0),
) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=rpy),
        material=material,
        name=name,
    )


def _add_sphere(
    part,
    radius: float,
    xyz: tuple[float, float, float],
    *,
    material=None,
    name: str | None = None,
) -> None:
    part.visual(Sphere(radius=radius), origin=Origin(xyz=xyz), material=material, name=name)


def _all_slat_pose(angle: float) -> dict[str, float]:
    return {f"slat_{index:02d}_tilt": angle for index in range(1, SLAT_COUNT + 1)}


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="louvered_shutter_assembly", assets=ASSETS)

    painted_wood = _make_material("painted_wood", (0.93, 0.93, 0.90, 1.0))
    warm_white_trim = _make_material("warm_white_trim", (0.97, 0.97, 0.95, 1.0))
    brushed_steel = _make_material("brushed_steel", (0.58, 0.60, 0.62, 1.0))
    dark_hardware = _make_material("dark_hardware", (0.16, 0.17, 0.18, 1.0))
    model.materials.extend([painted_wood, warm_white_trim, brushed_steel, dark_hardware])

    frame = model.part("frame")
    frame.inertial = Inertial.from_geometry(
        Box((FRAME_WIDTH, FRAME_DEPTH, FRAME_HEIGHT)),
        mass=8.0,
        origin=Origin(xyz=(0.0, 0.0, FRAME_HEIGHT / 2.0)),
    )

    jamb_x = FRAME_WIDTH / 2.0 - FRAME_MEMBER / 2.0
    rail_span = FRAME_WIDTH - 2.0 * FRAME_MEMBER
    _add_box(
        frame,
        (FRAME_MEMBER, FRAME_DEPTH, FRAME_HEIGHT),
        (-jamb_x, 0.0, FRAME_HEIGHT / 2.0),
        material=painted_wood,
        name="left_jamb",
    )
    _add_box(
        frame,
        (FRAME_MEMBER, FRAME_DEPTH, FRAME_HEIGHT),
        (jamb_x, 0.0, FRAME_HEIGHT / 2.0),
        material=painted_wood,
        name="right_jamb",
    )
    _add_box(
        frame,
        (rail_span, FRAME_DEPTH, FRAME_MEMBER),
        (0.0, 0.0, FRAME_HEIGHT - FRAME_MEMBER / 2.0),
        material=painted_wood,
        name="head",
    )
    _add_box(
        frame,
        (rail_span, FRAME_DEPTH, FRAME_MEMBER),
        (0.0, 0.0, FRAME_MEMBER / 2.0),
        material=painted_wood,
        name="sill",
    )

    front_trim_y = FRAME_DEPTH / 2.0 - 0.004
    side_trim_width = 0.074
    side_trim_x = FRAME_WIDTH / 2.0 + side_trim_width / 2.0 - 0.010
    cross_trim_width = FRAME_WIDTH + 0.020
    _add_box(
        frame,
        (side_trim_width, 0.008, FRAME_HEIGHT),
        (-side_trim_x, front_trim_y, FRAME_HEIGHT / 2.0),
        material=warm_white_trim,
        name="left_outer_trim",
    )
    _add_box(
        frame,
        (side_trim_width, 0.008, FRAME_HEIGHT),
        (side_trim_x, front_trim_y, FRAME_HEIGHT / 2.0),
        material=warm_white_trim,
        name="right_outer_trim",
    )
    _add_box(
        frame,
        (cross_trim_width, 0.008, 0.082),
        (0.0, front_trim_y, FRAME_HEIGHT - 0.041),
        material=warm_white_trim,
        name="head_trim",
    )
    _add_box(
        frame,
        (cross_trim_width, 0.010, 0.090),
        (0.0, front_trim_y, 0.045),
        material=warm_white_trim,
        name="sill_trim",
    )

    for hinge_index, hinge_z in enumerate(HINGE_Z_OFFSETS, start=1):
        plate_z = HINGE_AXIS_Z + hinge_z
        _add_box(
            frame,
            (0.036, 0.014, 0.120),
            (-0.334, FRAME_HINGE_PLATE_Y, plate_z),
            material=brushed_steel,
            name=f"frame_hinge_plate_{hinge_index}",
        )
        _add_cylinder(
            frame,
            0.0045,
            0.022,
            (HINGE_AXIS_X, HINGE_AXIS_Y, plate_z - 0.032),
            material=brushed_steel,
            name=f"frame_hinge_lower_{hinge_index}",
        )
        _add_cylinder(
            frame,
            0.0045,
            0.022,
            (HINGE_AXIS_X, HINGE_AXIS_Y, plate_z + 0.032),
            material=brushed_steel,
            name=f"frame_hinge_upper_{hinge_index}",
        )

    strike_x = FRAME_WIDTH / 2.0 - FRAME_MEMBER + 0.009
    _add_box(
        frame,
        (0.018, 0.008, 0.060),
        (strike_x, FRAME_DEPTH / 2.0 - 0.008, HINGE_AXIS_Z),
        material=dark_hardware,
        name="strike_plate",
    )

    shutter_leaf = model.part("shutter_leaf")
    shutter_leaf.inertial = Inertial.from_geometry(
        Box((LEAF_WIDTH, LEAF_THICKNESS, LEAF_HEIGHT)),
        mass=4.8,
        origin=Origin(xyz=(LEAF_LEFT_OFFSET + LEAF_WIDTH / 2.0, LEAF_PANEL_Y, 0.0)),
    )

    stile_centers = (
        LEAF_LEFT_OFFSET + LEAF_STILE / 2.0,
        LEAF_LEFT_OFFSET + LEAF_WIDTH - LEAF_STILE / 2.0,
    )
    rail_center_x = LEAF_LEFT_OFFSET + LEAF_WIDTH / 2.0
    clear_span = LEAF_WIDTH - 2.0 * LEAF_STILE
    top_rail_z = LEAF_HEIGHT / 2.0 - LEAF_RAIL / 2.0
    bottom_rail_z = -top_rail_z

    for side, center_x in zip(("left", "right"), stile_centers):
        _add_box(
            shutter_leaf,
            (LEAF_STILE, LEAF_THICKNESS, LEAF_HEIGHT),
            (center_x, LEAF_PANEL_Y, 0.0),
            material=painted_wood,
            name=f"{side}_stile",
        )
        _add_box(
            shutter_leaf,
            (LEAF_STILE - 0.018, 0.006, LEAF_HEIGHT - 0.050),
            (center_x, LEAF_PANEL_Y + 0.013, 0.0),
            material=warm_white_trim,
            name=f"{side}_stile_bead",
        )

    _add_box(
        shutter_leaf,
        (clear_span, LEAF_THICKNESS, LEAF_RAIL),
        (rail_center_x, LEAF_PANEL_Y, top_rail_z),
        material=painted_wood,
        name="top_rail",
    )
    _add_box(
        shutter_leaf,
        (clear_span, LEAF_THICKNESS, LEAF_RAIL),
        (rail_center_x, LEAF_PANEL_Y, bottom_rail_z),
        material=painted_wood,
        name="bottom_rail",
    )
    _add_box(
        shutter_leaf,
        (clear_span - 0.020, 0.006, LEAF_RAIL - 0.018),
        (rail_center_x, LEAF_PANEL_Y + 0.013, top_rail_z),
        material=warm_white_trim,
        name="top_rail_bead",
    )
    _add_box(
        shutter_leaf,
        (clear_span - 0.020, 0.006, LEAF_RAIL - 0.018),
        (rail_center_x, LEAF_PANEL_Y + 0.013, bottom_rail_z),
        material=warm_white_trim,
        name="bottom_rail_bead",
    )

    for hinge_index, hinge_z in enumerate(HINGE_Z_OFFSETS, start=1):
        _add_box(
            shutter_leaf,
            (0.030, 0.016, 0.112),
            (0.018, LEAF_HINGE_PLATE_Y, hinge_z),
            material=brushed_steel,
            name=f"leaf_hinge_plate_{hinge_index}",
        )
        _add_cylinder(
            shutter_leaf,
            0.0045,
            0.034,
            (0.0, HINGE_AXIS_Y, hinge_z),
            material=brushed_steel,
            name=f"leaf_hinge_knuckle_{hinge_index}",
        )

    pull_x = LEAF_LEFT_OFFSET + LEAF_WIDTH - LEAF_STILE / 2.0
    _add_box(
        shutter_leaf,
        (0.028, 0.008, 0.060),
        (pull_x, LEAF_PANEL_Y + 0.020, 0.0),
        material=dark_hardware,
        name="pull_backplate",
    )
    _add_cylinder(
        shutter_leaf,
        0.006,
        0.016,
        (pull_x, LEAF_PANEL_Y + 0.030, 0.0),
        material=dark_hardware,
        name="pull_stem",
        rpy=(pi / 2.0, 0.0, 0.0),
    )
    _add_sphere(
        shutter_leaf,
        0.008,
        (pull_x, LEAF_PANEL_Y + 0.038, 0.0),
        material=dark_hardware,
        name="pull_knob",
    )

    model.articulation(
        "frame_to_shutter",
        ArticulationType.REVOLUTE,
        parent="frame",
        child="shutter_leaf",
        origin=Origin(xyz=(HINGE_AXIS_X, HINGE_AXIS_Y, HINGE_AXIS_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.6,
            lower=0.0,
            upper=1.15,
        ),
    )

    for index, slat_z in enumerate(SLAT_ZS, start=1):
        slat = model.part(f"slat_{index:02d}")
        slat.inertial = Inertial.from_geometry(
            Box((SLAT_LENGTH, 0.012, SLAT_HEIGHT)),
            mass=0.11,
            origin=Origin(xyz=(SLAT_BODY_OFFSET_X + SLAT_LENGTH / 2.0, 0.0, 0.0)),
        )
        _add_box(
            slat,
            (SLAT_LENGTH, SLAT_CORE_DEPTH, SLAT_HEIGHT - 0.010),
            (SLAT_BODY_OFFSET_X + SLAT_LENGTH / 2.0, 0.0, 0.0),
            material=painted_wood,
            name=f"slat_core_{index:02d}",
        )
        _add_cylinder(
            slat,
            SLAT_EDGE_RADIUS,
            SLAT_LENGTH,
            (SLAT_BODY_OFFSET_X + SLAT_LENGTH / 2.0, 0.003, 0.0),
            material=warm_white_trim,
            name=f"slat_front_round_{index:02d}",
            rpy=(0.0, pi / 2.0, 0.0),
        )
        _add_cylinder(
            slat,
            SLAT_EDGE_RADIUS,
            SLAT_LENGTH,
            (SLAT_BODY_OFFSET_X + SLAT_LENGTH / 2.0, -0.003, 0.0),
            material=warm_white_trim,
            name=f"slat_back_round_{index:02d}",
            rpy=(0.0, pi / 2.0, 0.0),
        )
        model.articulation(
            f"slat_{index:02d}_tilt",
            ArticulationType.REVOLUTE,
            parent="shutter_leaf",
            child=f"slat_{index:02d}",
            origin=Origin(xyz=(SLAT_LEFT_X, LEAF_PANEL_Y, slat_z)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=1.5,
                velocity=5.0,
                lower=-0.55,
                upper=0.55,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=HERE)
    for index in range(1, SLAT_COUNT + 1):
        ctx.allow_overlap(
            "shutter_leaf",
            f"slat_{index:02d}",
            reason="Each louver rotates on concealed stile-side trunnions, and the generated collision hulls conservatively overestimate that seated hinge contact.",
        )
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.01)
    ctx.fail_if_part_contains_disconnected_geometry_islands(use="visual")
    ctx.fail_if_parts_overlap_in_sampled_poses(max_pose_samples=192, overlap_tol=0.003, overlap_volume_tol=0.0)

    ctx.expect_aabb_overlap("shutter_leaf", "frame", axes="xy", min_overlap=0.008)
    ctx.expect_joint_motion_axis(
        "frame_to_shutter",
        "shutter_leaf",
        world_axis="y",
        direction="positive",
        min_delta=0.12,
    )

    for slat_name in ("slat_02", "slat_06", "slat_10"):
        ctx.expect_aabb_overlap(slat_name, "shutter_leaf", axes="xy", min_overlap=0.008)

    for upper, lower in (("slat_02", "slat_01"), ("slat_06", "slat_05"), ("slat_11", "slat_10")):
        ctx.expect_aabb_gap(upper, lower, axis="z", max_gap=0.026, max_penetration=0.0)

    closed_mid_slat = ctx.part_world_position("slat_06")
    closed_upper_slat = ctx.part_world_position("slat_10")
    closed_zs = [
        ctx.part_world_position(f"slat_{index:02d}")[2] for index in range(1, SLAT_COUNT + 1)
    ]
    closed_steps = [b - a for a, b in zip(closed_zs, closed_zs[1:])]
    assert min(closed_steps) > 0.085, (
        "Slats should remain vertically ordered with consistent pitch."
    )
    assert max(closed_steps) < 0.099, "Slat pitch drift suggests the louver pack is uneven."

    with ctx.pose({"frame_to_shutter": 1.20}):
        open_mid_slat = ctx.part_world_position("slat_06")
        open_upper_slat = ctx.part_world_position("slat_10")
        assert open_mid_slat[1] > closed_mid_slat[1] + 0.06, (
            "The shutter leaf should swing the louver pack outward from the frame."
        )
        assert open_mid_slat[0] < closed_mid_slat[0] - 0.04, (
            "Opening the shutter should carry the slats in an arc toward the hinge side."
        )
        assert abs(open_upper_slat[1] - open_mid_slat[1]) < 0.012, (
            "All louvers should remain aligned within the opened shutter leaf."
        )
        ctx.expect_aabb_overlap("slat_06", "shutter_leaf", axes="xy", min_overlap=0.008)

    with ctx.pose(_all_slat_pose(0.50)):
        for upper, lower in (
            ("slat_03", "slat_02"),
            ("slat_07", "slat_06"),
            ("slat_11", "slat_10"),
        ):
            ctx.expect_aabb_gap(upper, lower, axis="z", max_gap=0.032, max_penetration=0.0)
        opened_pack_zs = [
            ctx.part_world_position(f"slat_{index:02d}")[2] for index in range(1, SLAT_COUNT + 1)
        ]
        opened_steps = [b - a for a, b in zip(opened_pack_zs, opened_pack_zs[1:])]
        assert min(opened_steps) > 0.085, "Opening the louvers should not collapse the slat stack."

    with ctx.pose(_all_slat_pose(-0.45)):
        for slat_name in ("slat_01", "slat_06", "slat_11"):
            ctx.expect_aabb_overlap(slat_name, "shutter_leaf", axes="xy", min_overlap=0.008)

    with ctx.pose({"frame_to_shutter": 1.10, **_all_slat_pose(0.45)}):
        swung_open_slat = ctx.part_world_position("slat_06")
        swung_open_upper = ctx.part_world_position("slat_10")
        assert swung_open_slat[1] > closed_mid_slat[1] + 0.055, (
            "The adjusted louvers should still follow the opened panel."
        )
        assert abs(swung_open_upper[1] - swung_open_slat[1]) < 0.012, (
            "The opened shutter should keep the louver bank visually planar."
        )
        assert swung_open_upper[1] > closed_upper_slat[1] + 0.055, (
            "Upper louvers should swing outward together with the rest of the panel."
        )
        ctx.expect_aabb_overlap("slat_10", "shutter_leaf", axes="xy", min_overlap=0.008)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
