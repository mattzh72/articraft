from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import cos, pi, sin

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _circle_profile(radius: float, *, segments: int = 48) -> list[tuple[float, float]]:
    return [
        (radius * cos((2.0 * pi * index) / segments), radius * sin((2.0 * pi * index) / segments))
        for index in range(segments)
    ]


def _sprocket_profile(
    outer_radius: float,
    tooth_radius: float,
    *,
    teeth: int = 22,
) -> list[tuple[float, float]]:
    points: list[tuple[float, float]] = []
    for index in range(teeth * 2):
        angle = (pi * index) / teeth
        radius = tooth_radius if index % 2 == 0 else outer_radius
        points.append((radius * cos(angle), radius * sin(angle)))
    return points


def _mirror_x(points: list[tuple[float, float, float]]) -> list[tuple[float, float, float]]:
    return [(-x, y, z) for x, y, z in points]


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="go_kart_live_rear_axle")

    frame_paint = model.material("frame_paint", rgba=(0.16, 0.17, 0.18, 1.0))
    bearing_cast = model.material("bearing_cast", rgba=(0.52, 0.54, 0.56, 1.0))
    axle_steel = model.material("axle_steel", rgba=(0.32, 0.34, 0.36, 1.0))
    machined_steel = model.material("machined_steel", rgba=(0.69, 0.70, 0.72, 1.0))
    sprocket_steel = model.material("sprocket_steel", rgba=(0.60, 0.61, 0.63, 1.0))

    rear_frame = model.part("rear_frame")
    rear_frame.inertial = Inertial.from_geometry(
        Box((0.86, 0.54, 0.26)),
        mass=22.0,
        origin=Origin(xyz=(0.0, 0.07, 0.13)),
    )

    left_rail = tube_from_spline_points(
        [
            (0.36, 0.34, 0.09),
            (0.355, 0.16, 0.09),
            (0.33, -0.02, 0.085),
            (0.30, -0.16, 0.08),
        ],
        radius=0.018,
        samples_per_segment=12,
        radial_segments=18,
        cap_ends=True,
    )
    right_rail = tube_from_spline_points(
        _mirror_x(
            [
                (0.36, 0.34, 0.09),
                (0.355, 0.16, 0.09),
                (0.33, -0.02, 0.085),
                (0.30, -0.16, 0.08),
            ]
        ),
        radius=0.018,
        samples_per_segment=12,
        radial_segments=18,
        cap_ends=True,
    )
    rear_frame.visual(_save_mesh("left_frame_rail", left_rail), material=frame_paint, name="left_frame_rail")
    rear_frame.visual(_save_mesh("right_frame_rail", right_rail), material=frame_paint, name="right_frame_rail")
    rear_frame.visual(
        Cylinder(radius=0.018, length=0.72),
        origin=Origin(xyz=(0.0, 0.14, 0.09), rpy=(0.0, pi / 2.0, 0.0)),
        material=frame_paint,
        name="front_cross_tube",
    )
    rear_frame.visual(
        Cylinder(radius=0.018, length=0.84),
        origin=Origin(xyz=(0.0, -0.14, 0.08), rpy=(0.0, pi / 2.0, 0.0)),
        material=frame_paint,
        name="rear_cross_tube",
    )
    rear_frame.visual(
        Box((0.12, 0.30, 0.03)),
        origin=Origin(xyz=(0.28, 0.0, 0.095)),
        material=frame_paint,
        name="left_pedestal_bridge",
    )
    rear_frame.visual(
        Box((0.12, 0.30, 0.03)),
        origin=Origin(xyz=(-0.28, 0.0, 0.095)),
        material=frame_paint,
        name="right_pedestal_bridge",
    )
    rear_frame.visual(
        Box((0.08, 0.12, 0.12)),
        origin=Origin(xyz=(0.28, 0.0, 0.17)),
        material=frame_paint,
        name="left_pedestal_web",
    )
    rear_frame.visual(
        Box((0.08, 0.12, 0.12)),
        origin=Origin(xyz=(-0.28, 0.0, 0.17)),
        material=frame_paint,
        name="right_pedestal_web",
    )
    rear_frame.visual(
        Box((0.12, 0.12, 0.02)),
        origin=Origin(xyz=(0.28, 0.0, 0.22)),
        material=frame_paint,
        name="left_mount_pad",
    )
    rear_frame.visual(
        Box((0.12, 0.12, 0.02)),
        origin=Origin(xyz=(-0.28, 0.0, 0.22)),
        material=frame_paint,
        name="right_mount_pad",
    )
    left_pillow_block = model.part("left_pillow_block")
    left_pillow_block.inertial = Inertial.from_geometry(
        Box((0.15, 0.09, 0.11)),
        mass=1.8,
        origin=Origin(xyz=(0.0, 0.0, 0.055)),
    )
    left_pillow_block.visual(
        Box((0.15, 0.09, 0.02)),
        origin=Origin(xyz=(0.0, 0.0, 0.01)),
        material=bearing_cast,
        name="base_foot",
    )
    left_pillow_block.visual(
        Box((0.07, 0.09, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, 0.027)),
        material=bearing_cast,
        name="bearing_saddle",
    )
    left_pillow_block.visual(
        Box((0.07, 0.014, 0.058)),
        origin=Origin(xyz=(0.0, -0.033, 0.056)),
        material=bearing_cast,
        name="left_cheek",
    )
    left_pillow_block.visual(
        Box((0.07, 0.014, 0.058)),
        origin=Origin(xyz=(0.0, 0.033, 0.056)),
        material=bearing_cast,
        name="right_cheek",
    )
    left_pillow_block.visual(
        Box((0.07, 0.052, 0.016)),
        origin=Origin(xyz=(0.0, 0.0, 0.093)),
        material=bearing_cast,
        name="bearing_cap",
    )
    left_pillow_block.visual(
        Cylinder(radius=0.008, length=0.02),
        origin=Origin(xyz=(0.065, 0.03, 0.02)),
        material=machined_steel,
        name="front_bolt_head",
    )
    left_pillow_block.visual(
        Cylinder(radius=0.008, length=0.02),
        origin=Origin(xyz=(-0.065, -0.03, 0.02)),
        material=machined_steel,
        name="rear_bolt_head",
    )

    right_pillow_block = model.part("right_pillow_block")
    right_pillow_block.inertial = Inertial.from_geometry(
        Box((0.15, 0.09, 0.11)),
        mass=1.8,
        origin=Origin(xyz=(0.0, 0.0, 0.055)),
    )
    right_pillow_block.visual(
        Box((0.15, 0.09, 0.02)),
        origin=Origin(xyz=(0.0, 0.0, 0.01)),
        material=bearing_cast,
        name="base_foot",
    )
    right_pillow_block.visual(
        Box((0.07, 0.09, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, 0.027)),
        material=bearing_cast,
        name="bearing_saddle",
    )
    right_pillow_block.visual(
        Box((0.07, 0.014, 0.058)),
        origin=Origin(xyz=(0.0, -0.033, 0.056)),
        material=bearing_cast,
        name="left_cheek",
    )
    right_pillow_block.visual(
        Box((0.07, 0.014, 0.058)),
        origin=Origin(xyz=(0.0, 0.033, 0.056)),
        material=bearing_cast,
        name="right_cheek",
    )
    right_pillow_block.visual(
        Box((0.07, 0.052, 0.016)),
        origin=Origin(xyz=(0.0, 0.0, 0.093)),
        material=bearing_cast,
        name="bearing_cap",
    )
    right_pillow_block.visual(
        Cylinder(radius=0.008, length=0.02),
        origin=Origin(xyz=(0.065, 0.03, 0.02)),
        material=machined_steel,
        name="front_bolt_head",
    )
    right_pillow_block.visual(
        Cylinder(radius=0.008, length=0.02),
        origin=Origin(xyz=(-0.065, -0.03, 0.02)),
        material=machined_steel,
        name="rear_bolt_head",
    )

    live_axle_assembly = model.part("live_axle_assembly")
    live_axle_assembly.inertial = Inertial.from_geometry(
        Cylinder(radius=0.06, length=0.96),
        mass=8.5,
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
    )
    live_axle_assembly.visual(
        Cylinder(radius=0.025, length=0.96),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=axle_steel,
        name="axle_shaft",
    )
    for side_name, hub_center in (("left", 0.432), ("right", -0.432)):
        sign = 1.0 if hub_center > 0.0 else -1.0
        live_axle_assembly.visual(
            Cylinder(radius=0.042, length=0.092),
            origin=Origin(xyz=(hub_center, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
            material=machined_steel,
            name=f"{side_name}_hub_sleeve",
        )
        live_axle_assembly.visual(
            Cylinder(radius=0.092, length=0.012),
            origin=Origin(xyz=(hub_center + sign * 0.05, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
            material=machined_steel,
            name=f"{side_name}_hub_flange",
        )
        for stud_index, (stud_y, stud_z) in enumerate(
            ((0.038, 0.0), (-0.038, 0.0), (0.0, 0.038), (0.0, -0.038))
        ):
            live_axle_assembly.visual(
                Cylinder(radius=0.0075, length=0.022),
                origin=Origin(
                    xyz=(hub_center + sign * 0.058, stud_y, stud_z),
                    rpy=(0.0, pi / 2.0, 0.0),
                ),
                material=axle_steel,
                name=f"{side_name}_stud_{stud_index}",
            )
        live_axle_assembly.visual(
            Box((0.026, 0.018, 0.022)),
            origin=Origin(xyz=(hub_center - sign * 0.028, 0.0, 0.051)),
            material=axle_steel,
            name=f"{side_name}_hub_clamp",
        )
        live_axle_assembly.visual(
            Cylinder(radius=0.033, length=0.014),
            origin=Origin(xyz=(hub_center - sign * 0.062, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
            material=axle_steel,
            name=f"{side_name}_locking_collar",
        )

    sprocket_mesh = _save_mesh(
        "drive_sprocket_plate",
        ExtrudeWithHolesGeometry(
            _sprocket_profile(0.095, 0.106, teeth=22),
            [_circle_profile(0.04, segments=36)],
            height=0.008,
            center=True,
        ).rotate_y(pi / 2.0),
    )
    live_axle_assembly.visual(
        sprocket_mesh,
        material=sprocket_steel,
        name="drive_sprocket",
    )
    live_axle_assembly.visual(
        Cylinder(radius=0.048, length=0.038),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=machined_steel,
        name="sprocket_carrier",
    )
    for arm_index, arm_angle in enumerate((0.0, 2.0 * pi / 3.0, 4.0 * pi / 3.0)):
        arm_y = cos(arm_angle) * 0.032
        arm_z = sin(arm_angle) * 0.032
        live_axle_assembly.visual(
            Box((0.008, 0.022, 0.07)),
            origin=Origin(xyz=(0.0, arm_y, arm_z), rpy=(arm_angle, 0.0, 0.0)),
            material=machined_steel,
            name=f"sprocket_spider_{arm_index}",
        )
    live_axle_assembly.visual(
        Box((0.018, 0.016, 0.022)),
        origin=Origin(xyz=(0.0, 0.0, 0.115)),
        material=axle_steel,
        name="sprocket_timing_tab",
    )

    model.articulation(
        "left_bearing_mount",
        ArticulationType.FIXED,
        parent=rear_frame,
        child=left_pillow_block,
        origin=Origin(xyz=(0.28, 0.0, 0.23)),
    )
    model.articulation(
        "right_bearing_mount",
        ArticulationType.FIXED,
        parent=rear_frame,
        child=right_pillow_block,
        origin=Origin(xyz=(-0.28, 0.0, 0.23)),
    )
    model.articulation(
        "rear_axle_spin",
        ArticulationType.CONTINUOUS,
        parent=rear_frame,
        child=live_axle_assembly,
        origin=Origin(xyz=(0.0, 0.0, 0.285)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=140.0, velocity=45.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.

    rear_frame = object_model.get_part("rear_frame")
    left_pillow_block = object_model.get_part("left_pillow_block")
    right_pillow_block = object_model.get_part("right_pillow_block")
    live_axle_assembly = object_model.get_part("live_axle_assembly")
    rear_axle_spin = object_model.get_articulation("rear_axle_spin")

    def elem_center(part, elem: str):
        aabb = ctx.part_element_world_aabb(part, elem=elem)
        if aabb is None:
            return None
        mins, maxs = aabb
        return tuple((mins[index] + maxs[index]) * 0.5 for index in range(3))

    ctx.check(
        "rear axle spins continuously about the lateral axis",
        rear_axle_spin.articulation_type == ArticulationType.CONTINUOUS
        and tuple(round(value, 6) for value in rear_axle_spin.axis) == (1.0, 0.0, 0.0)
        and rear_axle_spin.motion_limits is not None
        and rear_axle_spin.motion_limits.lower is None
        and rear_axle_spin.motion_limits.upper is None,
        details=(
            f"type={rear_axle_spin.articulation_type}, axis={rear_axle_spin.axis}, "
            f"limits={rear_axle_spin.motion_limits}"
        ),
    )

    ctx.expect_contact(
        left_pillow_block,
        rear_frame,
        elem_a="base_foot",
        elem_b="left_mount_pad",
        name="left pillow block is bolted to the left mount pad",
    )
    ctx.expect_contact(
        right_pillow_block,
        rear_frame,
        elem_a="base_foot",
        elem_b="right_mount_pad",
        name="right pillow block is bolted to the right mount pad",
    )

    ctx.expect_overlap(
        live_axle_assembly,
        left_pillow_block,
        axes="yz",
        elem_a="axle_shaft",
        min_overlap=0.045,
        name="axle shaft aligns with the left bearing opening",
    )
    ctx.expect_overlap(
        live_axle_assembly,
        right_pillow_block,
        axes="yz",
        elem_a="axle_shaft",
        min_overlap=0.045,
        name="axle shaft aligns with the right bearing opening",
    )

    left_flange_center = elem_center(live_axle_assembly, "left_hub_flange")
    right_flange_center = elem_center(live_axle_assembly, "right_hub_flange")
    sprocket_center = elem_center(live_axle_assembly, "drive_sprocket")
    ctx.check(
        "live axle carries hubs outboard and sprocket mid-span",
        left_flange_center is not None
        and right_flange_center is not None
        and sprocket_center is not None
        and left_flange_center[0] > 0.38
        and right_flange_center[0] < -0.38
        and abs(sprocket_center[0]) < 0.01,
        details=(
            f"left_flange_center={left_flange_center}, right_flange_center={right_flange_center}, "
            f"sprocket_center={sprocket_center}"
        ),
    )

    axle_origin = ctx.part_world_position(live_axle_assembly)
    left_clamp_rest = elem_center(live_axle_assembly, "left_hub_clamp")
    sprocket_tab_rest = elem_center(live_axle_assembly, "sprocket_timing_tab")
    with ctx.pose({rear_axle_spin: pi / 2.0}):
        left_clamp_quarter_turn = elem_center(live_axle_assembly, "left_hub_clamp")
        sprocket_tab_quarter_turn = elem_center(live_axle_assembly, "sprocket_timing_tab")

    ctx.check(
        "hub clamp rotates with the live axle",
        axle_origin is not None
        and left_clamp_rest is not None
        and left_clamp_quarter_turn is not None
        and left_clamp_rest[2] > axle_origin[2] + 0.03
        and left_clamp_quarter_turn[1] < -0.03
        and abs(left_clamp_quarter_turn[2] - axle_origin[2]) < 0.02,
        details=(
            f"axle_origin={axle_origin}, rest={left_clamp_rest}, "
            f"quarter_turn={left_clamp_quarter_turn}"
        ),
    )
    ctx.check(
        "sprocket timing tab rotates with the axle tube",
        axle_origin is not None
        and sprocket_tab_rest is not None
        and sprocket_tab_quarter_turn is not None
        and sprocket_tab_rest[2] > axle_origin[2] + 0.07
        and sprocket_tab_quarter_turn[1] < -0.08
        and abs(sprocket_tab_quarter_turn[2] - axle_origin[2]) < 0.03,
        details=(
            f"axle_origin={axle_origin}, rest={sprocket_tab_rest}, "
            f"quarter_turn={sprocket_tab_quarter_turn}"
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
