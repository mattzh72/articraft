from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="naval_twin_rail_missile_launcher")

    deck_gray = model.material("deck_gray", rgba=(0.31, 0.34, 0.36, 1.0))
    naval_gray = model.material("naval_gray", rgba=(0.62, 0.66, 0.69, 1.0))
    machinery_gray = model.material("machinery_gray", rgba=(0.42, 0.45, 0.48, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.20, 0.22, 0.24, 1.0))
    rail_metal = model.material("rail_metal", rgba=(0.72, 0.75, 0.78, 1.0))

    deck_base = model.part("deck_base")
    deck_base.visual(
        Box((4.2, 4.2, 0.14)),
        origin=Origin(xyz=(0.0, 0.0, 0.07)),
        material=deck_gray,
        name="deck_patch",
    )
    deck_base.visual(
        Cylinder(radius=1.00, length=0.18),
        origin=Origin(xyz=(0.0, 0.0, 0.23)),
        material=dark_steel,
        name="mount_flange",
    )
    deck_base.visual(
        Cylinder(radius=0.68, length=1.15),
        origin=Origin(xyz=(0.0, 0.0, 0.815)),
        material=naval_gray,
        name="lower_pedestal",
    )
    deck_base.visual(
        Cylinder(radius=0.52, length=0.76),
        origin=Origin(xyz=(0.0, 0.0, 1.735)),
        material=naval_gray,
        name="upper_pedestal",
    )
    deck_base.visual(
        Cylinder(radius=0.84, length=0.26),
        origin=Origin(xyz=(0.0, 0.0, 2.13)),
        material=machinery_gray,
        name="yaw_bearing",
    )
    deck_base.visual(
        Box((0.28, 0.72, 0.58)),
        origin=Origin(xyz=(0.52, 0.0, 0.43)),
        material=machinery_gray,
        name="buttress_pos_x",
    )
    deck_base.visual(
        Box((0.28, 0.72, 0.58)),
        origin=Origin(xyz=(-0.52, 0.0, 0.43)),
        material=machinery_gray,
        name="buttress_neg_x",
    )
    deck_base.visual(
        Box((0.72, 0.28, 0.58)),
        origin=Origin(xyz=(0.0, 0.52, 0.43)),
        material=machinery_gray,
        name="buttress_pos_y",
    )
    deck_base.visual(
        Box((0.72, 0.28, 0.58)),
        origin=Origin(xyz=(0.0, -0.52, 0.43)),
        material=machinery_gray,
        name="buttress_neg_y",
    )
    deck_base.inertial = Inertial.from_geometry(
        Box((4.2, 4.2, 2.26)),
        mass=7800.0,
        origin=Origin(xyz=(0.0, 0.0, 1.13)),
    )

    yaw_carriage = model.part("yaw_carriage")
    yaw_carriage.visual(
        Cylinder(radius=0.88, length=0.18),
        origin=Origin(xyz=(0.0, 0.0, 0.09)),
        material=dark_steel,
        name="slew_ring",
    )
    yaw_carriage.visual(
        Cylinder(radius=0.78, length=0.10),
        origin=Origin(xyz=(0.0, 0.0, 0.23)),
        material=machinery_gray,
        name="rotating_deck",
    )
    yaw_carriage.visual(
        Box((1.12, 1.48, 0.38)),
        origin=Origin(xyz=(0.10, 0.0, 0.35)),
        material=naval_gray,
        name="front_cradle",
    )
    yaw_carriage.visual(
        Box((0.84, 0.94, 0.48)),
        origin=Origin(xyz=(-0.72, 0.0, 0.38)),
        material=machinery_gray,
        name="equipment_trunk",
    )
    yaw_carriage.visual(
        Box((0.24, 0.44, 0.36)),
        origin=Origin(xyz=(0.20, 0.0, 0.18)),
        material=machinery_gray,
        name="forward_pedestal",
    )
    yaw_carriage.visual(
        Box((0.90, 0.70, 0.68)),
        origin=Origin(xyz=(-0.90, 0.0, 0.60)),
        material=machinery_gray,
        name="counterweight_box",
    )
    yaw_carriage.visual(
        Box((0.62, 0.76, 0.32)),
        origin=Origin(xyz=(-0.92, 0.0, 0.78)),
        material=machinery_gray,
        name="rear_hood",
    )
    yaw_carriage.visual(
        Box((0.54, 0.18, 0.92)),
        origin=Origin(xyz=(0.14, 0.73, 0.90)),
        material=naval_gray,
        name="left_trunnion_cheek",
    )
    yaw_carriage.visual(
        Box((0.54, 0.18, 0.92)),
        origin=Origin(xyz=(0.14, -0.73, 0.90)),
        material=naval_gray,
        name="right_trunnion_cheek",
    )
    yaw_carriage.visual(
        Box((0.36, 1.30, 0.18)),
        origin=Origin(xyz=(-0.02, 0.0, 1.22)),
        material=naval_gray,
        name="upper_bridge",
    )
    yaw_carriage.visual(
        Cylinder(radius=0.18, length=0.16),
        origin=Origin(xyz=(0.10, 0.90, 0.92), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="left_trunnion_cap",
    )
    yaw_carriage.visual(
        Cylinder(radius=0.18, length=0.16),
        origin=Origin(xyz=(0.10, -0.90, 0.92), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="right_trunnion_cap",
    )
    yaw_carriage.inertial = Inertial.from_geometry(
        Box((2.40, 1.80, 1.42)),
        mass=3600.0,
        origin=Origin(xyz=(-0.15, 0.0, 0.71)),
    )

    rail_frame = model.part("rail_frame")
    rail_frame.visual(
        Cylinder(radius=0.10, length=1.28),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="trunnion_shaft",
    )
    rail_frame.visual(
        Box((0.34, 0.92, 0.28)),
        origin=Origin(xyz=(-0.02, 0.0, 0.0)),
        material=machinery_gray,
        name="pivot_block",
    )
    rail_frame.visual(
        Box((0.70, 0.28, 0.16)),
        origin=Origin(xyz=(0.40, 0.0, -0.18)),
        material=dark_steel,
        name="elevation_drive_pack",
    )
    rail_frame.visual(
        Box((4.20, 0.16, 0.18)),
        origin=Origin(xyz=(2.70, 0.48, 0.10)),
        material=naval_gray,
        name="left_girder",
    )
    rail_frame.visual(
        Box((4.20, 0.16, 0.18)),
        origin=Origin(xyz=(2.70, -0.48, 0.10)),
        material=naval_gray,
        name="right_girder",
    )
    rail_frame.visual(
        Box((0.94, 1.08, 0.18)),
        origin=Origin(xyz=(0.28, 0.0, 0.10)),
        material=naval_gray,
        name="rear_crossbeam",
    )
    rail_frame.visual(
        Box((0.26, 0.98, 0.12)),
        origin=Origin(xyz=(2.55, 0.0, 0.16)),
        material=naval_gray,
        name="mid_crossbeam",
    )
    rail_frame.visual(
        Box((0.32, 1.06, 0.16)),
        origin=Origin(xyz=(4.55, 0.0, 0.16)),
        material=naval_gray,
        name="front_crossbeam",
    )

    for prefix, rail_y in (("left", 0.48), ("right", -0.48)):
        for index, stanchion_x in enumerate((0.95, 2.60, 4.25)):
            rail_frame.visual(
                Box((0.16, 0.10, 0.24)),
                origin=Origin(xyz=(stanchion_x, rail_y, 0.24)),
                material=machinery_gray,
                name=f"{prefix}_stanchion_{index}",
            )
        rail_frame.visual(
            Box((4.65, 0.07, 0.08)),
            origin=Origin(xyz=(3.025, rail_y, 0.28)),
            material=rail_metal,
            name=f"{prefix}_launch_rail",
        )
        rail_frame.visual(
            Box((4.15, 0.012, 0.12)),
            origin=Origin(xyz=(3.20, rail_y - 0.028, 0.31)),
            material=rail_metal,
            name=f"{prefix}_rail_inner_guide",
        )
        rail_frame.visual(
            Box((4.15, 0.012, 0.12)),
            origin=Origin(xyz=(3.20, rail_y + 0.028, 0.31)),
            material=rail_metal,
            name=f"{prefix}_rail_outer_guide",
        )
        rail_frame.visual(
            Box((0.32, 0.12, 0.10)),
            origin=Origin(xyz=(0.78, rail_y, 0.31)),
            material=dark_steel,
            name=f"{prefix}_hold_down_shoe",
        )
        rail_frame.visual(
            Box((0.16, 0.16, 0.14)),
            origin=Origin(xyz=(5.42, rail_y, 0.32)),
            material=dark_steel,
            name=f"{prefix}_muzzle_stop",
        )

    rail_frame.inertial = Inertial.from_geometry(
        Box((5.70, 1.30, 0.64)),
        mass=1900.0,
        origin=Origin(xyz=(2.85, 0.0, 0.16)),
    )

    model.articulation(
        "pedestal_yaw",
        ArticulationType.REVOLUTE,
        parent=deck_base,
        child=yaw_carriage,
        origin=Origin(xyz=(0.0, 0.0, 2.26)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=90000.0,
            velocity=0.45,
            lower=-math.pi,
            upper=math.pi,
        ),
    )
    model.articulation(
        "frame_elevation",
        ArticulationType.REVOLUTE,
        parent=yaw_carriage,
        child=rail_frame,
        origin=Origin(xyz=(0.10, 0.0, 0.92)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=70000.0,
            velocity=0.40,
            lower=0.0,
            upper=math.radians(68.0),
        ),
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

    deck_base = object_model.get_part("deck_base")
    yaw_carriage = object_model.get_part("yaw_carriage")
    rail_frame = object_model.get_part("rail_frame")
    pedestal_yaw = object_model.get_articulation("pedestal_yaw")
    frame_elevation = object_model.get_articulation("frame_elevation")

    def aabb_center(aabb):
        if aabb is None:
            return None
        return tuple((aabb[0][i] + aabb[1][i]) * 0.5 for i in range(3))

    ctx.check(
        "launcher parts exist",
        all(part is not None for part in (deck_base, yaw_carriage, rail_frame)),
        details="Expected deck base, yaw carriage, and rail frame parts to exist.",
    )
    ctx.check(
        "pedestal yaw is vertical revolute",
        pedestal_yaw.articulation_type == ArticulationType.REVOLUTE
        and pedestal_yaw.axis == (0.0, 0.0, 1.0)
        and pedestal_yaw.motion_limits is not None
        and pedestal_yaw.motion_limits.lower is not None
        and pedestal_yaw.motion_limits.upper is not None,
        details=f"type={pedestal_yaw.articulation_type}, axis={pedestal_yaw.axis}, limits={pedestal_yaw.motion_limits}",
    )
    ctx.check(
        "frame elevation is transverse revolute",
        frame_elevation.articulation_type == ArticulationType.REVOLUTE
        and frame_elevation.axis == (0.0, -1.0, 0.0)
        and frame_elevation.motion_limits is not None
        and frame_elevation.motion_limits.lower == 0.0
        and frame_elevation.motion_limits.upper is not None
        and frame_elevation.motion_limits.upper > 1.0,
        details=f"type={frame_elevation.articulation_type}, axis={frame_elevation.axis}, limits={frame_elevation.motion_limits}",
    )

    with ctx.pose({frame_elevation: 0.0, pedestal_yaw: 0.0}):
        ctx.expect_contact(
            yaw_carriage,
            deck_base,
            elem_a="slew_ring",
            elem_b="yaw_bearing",
            contact_tol=0.002,
            name="yaw carriage sits on bearing ring",
        )
        ctx.expect_contact(
            rail_frame,
            yaw_carriage,
            elem_a="trunnion_shaft",
            elem_b="left_trunnion_cheek",
            contact_tol=0.002,
            name="left trunnion supports elevation frame",
        )
        ctx.expect_contact(
            rail_frame,
            yaw_carriage,
            elem_a="trunnion_shaft",
            elem_b="right_trunnion_cheek",
            contact_tol=0.002,
            name="right trunnion supports elevation frame",
        )

    rest_left_rail = ctx.part_element_world_aabb(rail_frame, elem="left_launch_rail")
    with ctx.pose({frame_elevation: math.radians(60.0)}):
        raised_left_rail = ctx.part_element_world_aabb(rail_frame, elem="left_launch_rail")
    rest_left_center = aabb_center(rest_left_rail)
    raised_left_center = aabb_center(raised_left_rail)
    ctx.check(
        "frame elevation raises rail nose",
        rest_left_center is not None
        and raised_left_center is not None
        and raised_left_center[2] > rest_left_center[2] + 2.0
        and raised_left_center[0] < rest_left_center[0] - 0.9,
        details=f"rest_center={rest_left_center}, raised_center={raised_left_center}",
    )

    with ctx.pose({pedestal_yaw: 0.0, frame_elevation: 0.0}):
        fore_left_rail = ctx.part_element_world_aabb(rail_frame, elem="left_launch_rail")
    with ctx.pose({pedestal_yaw: math.pi / 2.0, frame_elevation: 0.0}):
        traversed_left_rail = ctx.part_element_world_aabb(rail_frame, elem="left_launch_rail")
    fore_left_center = aabb_center(fore_left_rail)
    traversed_left_center = aabb_center(traversed_left_rail)
    ctx.check(
        "pedestal yaw swings launcher in plan",
        fore_left_center is not None
        and traversed_left_center is not None
        and fore_left_center[0] > 2.0
        and abs(traversed_left_center[0]) < 1.0
        and traversed_left_center[1] > 2.0,
        details=f"fore_center={fore_left_center}, traversed_center={traversed_left_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
