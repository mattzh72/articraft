from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    ExtrudeGeometry,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)

ASSETS = AssetContext.from_script(__file__)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="singleleaf_drawbridge_utility", assets=ASSETS)

    concrete = model.material("concrete", rgba=(0.57, 0.58, 0.60, 1.0))
    painted_steel = model.material("painted_steel", rgba=(0.19, 0.30, 0.37, 1.0))
    dark_non_skid = model.material("dark_non_skid", rgba=(0.16, 0.16, 0.15, 1.0))
    safety_yellow = model.material("safety_yellow", rgba=(0.82, 0.70, 0.18, 1.0))
    bearing_black = model.material("bearing_black", rgba=(0.08, 0.09, 0.10, 1.0))
    fastener_steel = model.material("fastener_steel", rgba=(0.56, 0.58, 0.60, 1.0))

    side_girder_profile = [
        (-0.040, -0.040),
        (0.070, -0.040),
        (0.260, -0.034),
        (0.500, -0.027),
        (0.720, -0.020),
        (0.720, 0.002),
        (-0.040, 0.002),
    ]
    side_girder_geom = ExtrudeGeometry.centered(side_girder_profile, 0.024)
    side_girder_geom.rotate_x(math.pi / 2.0)
    side_girder_mesh = mesh_from_geometry(side_girder_geom, ASSETS.mesh_path("leaf_side_girder.obj"))

    bearing_shell_geom = LatheGeometry.from_shell_profiles(
        outer_profile=[
            (0.038, -0.014),
            (0.046, -0.010),
            (0.046, 0.010),
            (0.038, 0.014),
        ],
        inner_profile=[
            (0.033, -0.014),
            (0.033, 0.014),
        ],
        segments=42,
        start_cap="flat",
        end_cap="flat",
    )
    bearing_shell_geom.rotate_x(math.pi / 2.0)
    bearing_shell_mesh = mesh_from_geometry(
        bearing_shell_geom,
        ASSETS.mesh_path("bearing_shell.obj"),
    )

    abutment_frame = model.part("abutment_frame")
    abutment_frame.visual(
        Box((0.42, 0.62, 0.07)),
        origin=Origin(xyz=(-0.12, 0.0, 0.035)),
        material=concrete,
        name="base_block",
    )
    abutment_frame.visual(
        Box((0.14, 0.42, 0.025)),
        origin=Origin(xyz=(-0.095, 0.0, 0.0825)),
        material=concrete,
        name="deck_slab",
    )
    abutment_frame.visual(
        Box((0.08, 0.08, 0.18)),
        origin=Origin(xyz=(-0.09, 0.26, 0.16)),
        material=painted_steel,
        name="left_cheek_wall",
    )
    abutment_frame.visual(
        Box((0.08, 0.08, 0.18)),
        origin=Origin(xyz=(-0.09, -0.26, 0.16)),
        material=painted_steel,
        name="right_cheek_wall",
    )
    abutment_frame.visual(
        Box((0.18, 0.44, 0.05)),
        origin=Origin(xyz=(-0.16, 0.0, 0.205)),
        material=painted_steel,
        name="rear_tie_beam",
    )
    abutment_frame.visual(
        Box((0.12, 0.22, 0.012)),
        origin=Origin(xyz=(-0.18, 0.0, 0.076)),
        material=bearing_black,
        name="service_cover",
    )
    abutment_frame.visual(
        Box((0.10, 0.03, 0.03)),
        origin=Origin(xyz=(-0.10, 0.185, 0.110)),
        material=safety_yellow,
        name="left_edge_guard",
    )
    abutment_frame.visual(
        Box((0.10, 0.03, 0.03)),
        origin=Origin(xyz=(-0.10, -0.185, 0.110)),
        material=safety_yellow,
        name="right_edge_guard",
    )
    for index, y_pos in enumerate((-0.08, 0.08), start=1):
        abutment_frame.visual(
            Cylinder(radius=0.008, length=0.008),
            origin=Origin(xyz=(-0.18, y_pos, 0.086)),
            material=fastener_steel,
            name=f"service_bolt_{index}",
        )
    abutment_frame.inertial = Inertial.from_geometry(
        Box((0.42, 0.62, 0.23)),
        mass=22.0,
        origin=Origin(xyz=(-0.12, 0.0, 0.115)),
    )

    def build_bearing(name: str) -> object:
        bearing = model.part(name)
        bearing.visual(
            Box((0.08, 0.07, 0.036)),
            origin=Origin(xyz=(-0.01, 0.0, -0.057)),
            material=painted_steel,
            name="mount_block",
        )
        bearing.visual(
            bearing_shell_mesh,
            material=bearing_black,
            name="bearing_shell",
        )
        for index, x_pos in enumerate((-0.026, 0.006), start=1):
            bearing.visual(
                Cylinder(radius=0.006, length=0.008),
                origin=Origin(xyz=(x_pos, 0.0, -0.043)),
                material=fastener_steel,
                name=f"cap_bolt_{index}",
            )
        bearing.inertial = Inertial.from_geometry(
            Box((0.10, 0.09, 0.10)),
            mass=0.9,
            origin=Origin(xyz=(0.01, 0.0, -0.025)),
        )
        return bearing

    left_bearing = build_bearing("left_bearing")
    right_bearing = build_bearing("right_bearing")

    model.articulation(
        "frame_to_left_bearing",
        ArticulationType.FIXED,
        parent=abutment_frame,
        child=left_bearing,
        origin=Origin(xyz=(0.0, 0.255, 0.147)),
    )
    model.articulation(
        "frame_to_right_bearing",
        ArticulationType.FIXED,
        parent=abutment_frame,
        child=right_bearing,
        origin=Origin(xyz=(0.0, -0.255, 0.147)),
    )

    bridge_leaf = model.part("bridge_leaf")
    bridge_leaf.visual(
        Box((0.72, 0.40, 0.018)),
        origin=Origin(xyz=(0.36, 0.0, 0.011)),
        material=painted_steel,
        name="deck_plate",
    )
    bridge_leaf.visual(
        Box((0.68, 0.30, 0.004)),
        origin=Origin(xyz=(0.38, 0.0, 0.022)),
        material=dark_non_skid,
        name="roadway_surface",
    )
    bridge_leaf.visual(
        side_girder_mesh,
        origin=Origin(xyz=(0.0, 0.188, 0.0)),
        material=painted_steel,
        name="left_side_girder",
    )
    bridge_leaf.visual(
        side_girder_mesh,
        origin=Origin(xyz=(0.0, -0.188, 0.0)),
        material=painted_steel,
        name="right_side_girder",
    )
    bridge_leaf.visual(
        Box((0.11, 0.32, 0.042)),
        origin=Origin(xyz=(0.01, 0.0, -0.019)),
        material=painted_steel,
        name="hinge_beam",
    )
    bridge_leaf.visual(
        Box((0.42, 0.05, 0.040)),
        origin=Origin(xyz=(0.23, 0.0, -0.018)),
        material=painted_steel,
        name="center_stiffener",
    )
    for index, x_pos in enumerate((0.16, 0.34, 0.54), start=1):
        bridge_leaf.visual(
            Box((0.03, 0.34, 0.028)),
            origin=Origin(xyz=(x_pos, 0.0, -0.012)),
            material=painted_steel,
            name=f"cross_stiffener_{index}",
        )
    bridge_leaf.visual(
        Box((0.58, 0.03, 0.036)),
        origin=Origin(xyz=(0.39, 0.185, 0.038)),
        material=safety_yellow,
        name="left_curb_beam",
    )
    bridge_leaf.visual(
        Box((0.58, 0.03, 0.036)),
        origin=Origin(xyz=(0.39, -0.185, 0.038)),
        material=safety_yellow,
        name="right_curb_beam",
    )
    bridge_leaf.visual(
        Box((0.03, 0.40, 0.034)),
        origin=Origin(xyz=(0.705, 0.0, -0.015)),
        material=painted_steel,
        name="nose_beam",
    )
    bridge_leaf.visual(
        Cylinder(radius=0.028, length=0.066),
        origin=Origin(xyz=(0.0, 0.208, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=painted_steel,
        name="left_trunnion_shaft",
    )
    bridge_leaf.visual(
        Cylinder(radius=0.028, length=0.066),
        origin=Origin(xyz=(0.0, -0.208, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=painted_steel,
        name="right_trunnion_shaft",
    )
    bridge_leaf.visual(
        Cylinder(radius=0.038, length=0.024),
        origin=Origin(xyz=(0.0, 0.229, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=painted_steel,
        name="left_trunnion_collar",
    )
    bridge_leaf.visual(
        Cylinder(radius=0.038, length=0.024),
        origin=Origin(xyz=(0.0, -0.229, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=painted_steel,
        name="right_trunnion_collar",
    )
    bridge_leaf.visual(
        Box((0.08, 0.030, 0.050)),
        origin=Origin(xyz=(0.005, 0.175, -0.012)),
        material=painted_steel,
        name="left_hinge_gusset",
    )
    bridge_leaf.visual(
        Box((0.08, 0.030, 0.050)),
        origin=Origin(xyz=(0.005, -0.175, -0.012)),
        material=painted_steel,
        name="right_hinge_gusset",
    )
    for index, (x_pos, y_pos) in enumerate(
        ((-0.005, -0.120), (-0.005, 0.120), (0.030, -0.120), (0.030, 0.120)),
        start=1,
    ):
        bridge_leaf.visual(
            Cylinder(radius=0.007, length=0.006),
            origin=Origin(xyz=(x_pos, y_pos, 0.023)),
            material=fastener_steel,
            name=f"hinge_fastener_{index}",
        )
    bridge_leaf.inertial = Inertial.from_geometry(
        Box((0.76, 0.46, 0.12)),
        mass=9.0,
        origin=Origin(xyz=(0.34, 0.0, 0.0)),
    )

    model.articulation(
        "frame_to_leaf",
        ArticulationType.REVOLUTE,
        parent=abutment_frame,
        child=bridge_leaf,
        origin=Origin(xyz=(0.0, 0.0, 0.147)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=32.0,
            velocity=0.8,
            lower=0.0,
            upper=math.radians(72.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    abutment_frame = object_model.get_part("abutment_frame")
    left_bearing = object_model.get_part("left_bearing")
    right_bearing = object_model.get_part("right_bearing")
    bridge_leaf = object_model.get_part("bridge_leaf")
    frame_to_leaf = object_model.get_articulation("frame_to_leaf")

    left_cheek_wall = abutment_frame.get_visual("left_cheek_wall")
    right_cheek_wall = abutment_frame.get_visual("right_cheek_wall")
    deck_slab = abutment_frame.get_visual("deck_slab")
    left_mount_block = left_bearing.get_visual("mount_block")
    right_mount_block = right_bearing.get_visual("mount_block")
    left_bearing_shell = left_bearing.get_visual("bearing_shell")
    right_bearing_shell = right_bearing.get_visual("bearing_shell")
    deck_plate = bridge_leaf.get_visual("deck_plate")
    nose_beam = bridge_leaf.get_visual("nose_beam")
    left_trunnion_collar = bridge_leaf.get_visual("left_trunnion_collar")
    right_trunnion_collar = bridge_leaf.get_visual("right_trunnion_collar")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Preferred default QC stack:
    # 1) likely-failure broad-part floating check for isolated parts
    ctx.fail_if_isolated_parts()
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

    ctx.expect_contact(
        left_bearing,
        abutment_frame,
        elem_a=left_mount_block,
        elem_b=left_cheek_wall,
        name="left_bearing_mounts_to_frame",
    )
    ctx.expect_contact(
        right_bearing,
        abutment_frame,
        elem_a=right_mount_block,
        elem_b=right_cheek_wall,
        name="right_bearing_mounts_to_frame",
    )
    ctx.expect_contact(
        bridge_leaf,
        left_bearing,
        elem_a=left_trunnion_collar,
        elem_b=left_bearing_shell,
        name="left_trunnion_seats_in_bearing",
    )
    ctx.expect_contact(
        bridge_leaf,
        right_bearing,
        elem_a=right_trunnion_collar,
        elem_b=right_bearing_shell,
        name="right_trunnion_seats_in_bearing",
    )
    ctx.expect_overlap(
        bridge_leaf,
        abutment_frame,
        axes="y",
        min_overlap=0.38,
        elem_a=deck_plate,
        elem_b=deck_slab,
        name="leaf_aligns_over_abutment_width",
    )
    ctx.expect_origin_distance(
        left_bearing,
        right_bearing,
        axes="y",
        min_dist=0.48,
        max_dist=0.54,
        name="bearing_stance_width",
    )
    ctx.fail_if_articulation_overlaps(max_pose_samples=24, name="leaf_motion_clearance")

    rest_nose_aabb = ctx.part_element_world_aabb(bridge_leaf, elem=nose_beam)
    assert rest_nose_aabb is not None

    with ctx.pose({frame_to_leaf: math.radians(65.0)}):
        open_nose_aabb = ctx.part_element_world_aabb(bridge_leaf, elem=nose_beam)
        assert open_nose_aabb is not None
        ctx.expect_contact(
            bridge_leaf,
            left_bearing,
            elem_a=left_trunnion_collar,
            elem_b=left_bearing_shell,
            name="left_trunnion_stays_seated_when_open",
        )
        ctx.expect_contact(
            bridge_leaf,
            right_bearing,
            elem_a=right_trunnion_collar,
            elem_b=right_bearing_shell,
            name="right_trunnion_stays_seated_when_open",
        )
        ctx.check(
            "leaf_tip_raises_clear_of_channel",
            open_nose_aabb[1][2] > rest_nose_aabb[1][2] + 0.45,
            details=(
                f"Expected open tip z > {rest_nose_aabb[1][2] + 0.45:.3f}, "
                f"got {open_nose_aabb[1][2]:.3f}"
            ),
        )
        ctx.check(
            "leaf_tip_swings_back_about_hinge",
            open_nose_aabb[1][0] < rest_nose_aabb[1][0] - 0.25,
            details=(
                f"Expected open tip x < {rest_nose_aabb[1][0] - 0.25:.3f}, "
                f"got {open_nose_aabb[1][0]:.3f}"
            ),
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
