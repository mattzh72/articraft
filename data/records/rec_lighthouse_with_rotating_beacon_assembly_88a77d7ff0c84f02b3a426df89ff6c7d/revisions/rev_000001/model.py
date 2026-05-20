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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="harbor_beacon_tower")

    # Real-world-ish dimensions for a squat navigation beacon.
    base_plinth_h = 0.18
    wall_h = 2.50
    roof_h = 0.10
    roof_top_z = base_plinth_h + wall_h + roof_h
    base_half = 0.95
    top_half = 0.72
    wall_t = 0.10
    wall_span = 1.82
    wall_taper = math.atan((base_half - top_half) / wall_h)

    door_opening_w = 0.56
    door_panel_w = 0.50
    door_h = 0.86
    door_t = 0.030
    door_opening_z0 = 1.50
    door_opening_z1 = 2.46
    door_z0 = 1.55
    door_z1 = door_z0 + door_h
    door_mid_z = 0.5 * (door_z0 + door_z1)
    side_jamb_w = 0.5 * (wall_span - door_opening_w)
    door_hinge_y = 0.25

    pedestal_flange_h = 0.04
    pedestal_h = 0.22
    pedestal_top_z = roof_top_z + pedestal_flange_h + pedestal_h

    def side_face_at(z: float) -> float:
        rel = min(max((z - base_plinth_h) / wall_h, 0.0), 1.0)
        return base_half + (top_half - base_half) * rel

    model.material("tower_shell", rgba=(0.93, 0.93, 0.88, 1.0))
    model.material("roof_paint", rgba=(0.48, 0.16, 0.14, 1.0))
    model.material("weathered_steel", rgba=(0.42, 0.43, 0.45, 1.0))
    model.material("door_paint", rgba=(0.33, 0.38, 0.43, 1.0))
    model.material("lantern_glass", rgba=(0.82, 0.68, 0.22, 0.70))
    model.material("dark_trim", rgba=(0.18, 0.18, 0.20, 1.0))

    tower = model.part("tower")
    tower.visual(
        Box((2.10, 2.10, base_plinth_h)),
        origin=Origin(xyz=(0.0, 0.0, 0.5 * base_plinth_h)),
        material="tower_shell",
        name="base_plinth",
    )

    # Front and rear tapered wall planes.
    tower.visual(
        Box((wall_span, wall_t, wall_h / math.cos(wall_taper))),
        origin=Origin(
            xyz=(0.0, side_face_at(base_plinth_h + 0.5 * wall_h) - 0.5 * wall_t, base_plinth_h + 0.5 * wall_h),
            rpy=(wall_taper, 0.0, 0.0),
        ),
        material="tower_shell",
        name="front_wall",
    )
    tower.visual(
        Box((wall_span, wall_t, wall_h / math.cos(wall_taper))),
        origin=Origin(
            xyz=(0.0, -side_face_at(base_plinth_h + 0.5 * wall_h) + 0.5 * wall_t, base_plinth_h + 0.5 * wall_h),
            rpy=(-wall_taper, 0.0, 0.0),
        ),
        material="tower_shell",
        name="rear_wall",
    )

    # Left wall stays whole; right wall is split around the equipment door opening.
    tower.visual(
        Box((wall_t, wall_span, wall_h / math.cos(wall_taper))),
        origin=Origin(
            xyz=(-side_face_at(base_plinth_h + 0.5 * wall_h) + 0.5 * wall_t, 0.0, base_plinth_h + 0.5 * wall_h),
            rpy=(0.0, wall_taper, 0.0),
        ),
        material="tower_shell",
        name="left_wall",
    )

    right_lower_h = door_opening_z0 - base_plinth_h
    right_upper_h = base_plinth_h + wall_h - door_opening_z1
    tower.visual(
        Box((wall_t, wall_span, right_lower_h / math.cos(wall_taper))),
        origin=Origin(
            xyz=(side_face_at(base_plinth_h + 0.5 * right_lower_h) - 0.5 * wall_t, 0.0, base_plinth_h + 0.5 * right_lower_h),
            rpy=(0.0, -wall_taper, 0.0),
        ),
        material="tower_shell",
        name="right_wall_lower",
    )
    tower.visual(
        Box((wall_t, wall_span, right_upper_h / math.cos(wall_taper))),
        origin=Origin(
            xyz=(
                side_face_at(door_opening_z1 + 0.5 * right_upper_h) - 0.5 * wall_t,
                0.0,
                door_opening_z1 + 0.5 * right_upper_h,
            ),
            rpy=(0.0, -wall_taper, 0.0),
        ),
        material="tower_shell",
        name="right_wall_upper",
    )
    tower.visual(
        Box((wall_t, side_jamb_w, (door_opening_z1 - door_opening_z0) / math.cos(wall_taper))),
        origin=Origin(
            xyz=(side_face_at(door_mid_z) - 0.5 * wall_t, 0.5 * (door_opening_w + side_jamb_w), door_mid_z),
            rpy=(0.0, -wall_taper, 0.0),
        ),
        material="tower_shell",
        name="door_frame_forward_jamb",
    )
    tower.visual(
        Box((wall_t, side_jamb_w, (door_opening_z1 - door_opening_z0) / math.cos(wall_taper))),
        origin=Origin(
            xyz=(side_face_at(door_mid_z) - 0.5 * wall_t, -0.5 * (door_opening_w + side_jamb_w), door_mid_z),
            rpy=(0.0, -wall_taper, 0.0),
        ),
        material="tower_shell",
        name="door_frame_aft_jamb",
    )

    # Roof cap and lantern pedestal.
    tower.visual(
        Box((1.58, 1.58, roof_h)),
        origin=Origin(xyz=(0.0, 0.0, base_plinth_h + wall_h + 0.5 * roof_h)),
        material="roof_paint",
        name="roof_cap",
    )
    tower.visual(
        Cylinder(radius=0.28, length=pedestal_flange_h),
        origin=Origin(xyz=(0.0, 0.0, roof_top_z + 0.5 * pedestal_flange_h)),
        material="weathered_steel",
        name="pedestal_flange",
    )
    tower.visual(
        Cylinder(radius=0.18, length=pedestal_h),
        origin=Origin(xyz=(0.0, 0.0, roof_top_z + pedestal_flange_h + 0.5 * pedestal_h)),
        material="weathered_steel",
        name="pedestal_shaft",
    )

    door = model.part("equipment_door")
    door.visual(
        Box((door_t, door_panel_w, door_h)),
        origin=Origin(
            xyz=(0.0, -0.5 * door_panel_w, 0.0),
            rpy=(0.0, -wall_taper, 0.0),
        ),
        material="door_paint",
        name="door_leaf",
    )
    door.visual(
        Box((0.008, door_panel_w - 0.10, door_h - 0.14)),
        origin=Origin(
            xyz=(0.5 * door_t + 0.004, -0.5 * door_panel_w, 0.0),
            rpy=(0.0, -wall_taper, 0.0),
        ),
        material="dark_trim",
        name="door_outer_skin",
    )
    door.visual(
        Box((0.030, 0.018, 0.16)),
        origin=Origin(
            xyz=(0.5 * door_t + 0.020, -door_panel_w + 0.10, -0.05),
            rpy=(0.0, -wall_taper, 0.0),
        ),
        material="weathered_steel",
        name="door_handle",
    )
    door.visual(
        Cylinder(radius=0.03, length=0.14),
        origin=Origin(xyz=(0.0, 0.0, 0.28)),
        material="weathered_steel",
        name="hinge_knuckle_upper",
    )
    door.visual(
        Cylinder(radius=0.03, length=0.14),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material="weathered_steel",
        name="hinge_knuckle_mid",
    )
    door.visual(
        Cylinder(radius=0.03, length=0.14),
        origin=Origin(xyz=(0.0, 0.0, -0.28)),
        material="weathered_steel",
        name="hinge_knuckle_lower",
    )

    lantern = model.part("lantern")
    lantern.visual(
        Cylinder(radius=0.26, length=0.05),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material="dark_trim",
        name="turntable_ring",
    )
    lantern.visual(
        Cylinder(radius=0.07, length=0.10),
        origin=Origin(xyz=(0.0, 0.0, 0.10)),
        material="weathered_steel",
        name="turntable_spindle",
    )
    lantern.visual(
        Cylinder(radius=0.14, length=0.24),
        origin=Origin(xyz=(0.0, 0.0, 0.22)),
        material="lantern_glass",
        name="optic_drum",
    )
    lantern.visual(
        Cylinder(radius=0.20, length=0.03),
        origin=Origin(xyz=(0.0, 0.0, 0.355)),
        material="roof_paint",
        name="lantern_hood",
    )
    lantern.visual(
        Cylinder(radius=0.06, length=0.06),
        origin=Origin(xyz=(0.0, 0.0, 0.40)),
        material="roof_paint",
        name="hood_cap",
    )
    lantern.visual(
        Cylinder(radius=0.018, length=0.31),
        origin=Origin(xyz=(0.0, 0.16, 0.205)),
        material="weathered_steel",
        name="guard_post_port",
    )
    lantern.visual(
        Cylinder(radius=0.018, length=0.31),
        origin=Origin(xyz=(0.0, -0.16, 0.205)),
        material="weathered_steel",
        name="guard_post_starboard",
    )
    lantern.visual(
        Box((0.10, 0.14, 0.14)),
        origin=Origin(xyz=(-0.18, 0.0, 0.20)),
        material="weathered_steel",
        name="rear_service_box",
    )

    model.articulation(
        "tower_to_lantern",
        ArticulationType.CONTINUOUS,
        parent=tower,
        child=lantern,
        origin=Origin(xyz=(0.0, 0.0, pedestal_top_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=20.0, velocity=8.0),
    )
    model.articulation(
        "tower_to_door",
        ArticulationType.REVOLUTE,
        parent=tower,
        child=door,
        origin=Origin(
            xyz=(
                side_face_at(door_mid_z) - 0.5 * wall_t - 0.5 * door_t + 0.02,
                door_hinge_y,
                door_mid_z,
            )
        ),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.0,
            lower=0.0,
            upper=0.55,
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

    tower = object_model.get_part("tower")
    lantern = object_model.get_part("lantern")
    door = object_model.get_part("equipment_door")
    lantern_joint = object_model.get_articulation("tower_to_lantern")
    door_joint = object_model.get_articulation("tower_to_door")

    ctx.check(
        "lantern articulation is continuous and vertical",
        lantern_joint.articulation_type == ArticulationType.CONTINUOUS
        and tuple(lantern_joint.axis) == (0.0, 0.0, 1.0),
        details=f"type={lantern_joint.articulation_type}, axis={lantern_joint.axis}",
    )
    ctx.check(
        "door articulation is vertical side hinge",
        door_joint.articulation_type == ArticulationType.REVOLUTE
        and tuple(door_joint.axis) == (0.0, 0.0, 1.0),
        details=f"type={door_joint.articulation_type}, axis={door_joint.axis}",
    )

    ctx.expect_contact(
        lantern,
        tower,
        elem_a="turntable_ring",
        elem_b="pedestal_shaft",
        contact_tol=1e-4,
        name="lantern turntable seats on pedestal",
    )
    ctx.expect_overlap(
        door,
        tower,
        axes="z",
        elem_a="door_leaf",
        min_overlap=0.80,
        name="door sits within upper wall height band",
    )
    ctx.expect_contact(
        door,
        tower,
        elem_a="hinge_knuckle_mid",
        elem_b="door_frame_forward_jamb",
        contact_tol=1e-4,
        name="door hinge knuckle stays mounted to tower jamb",
    )

    closed_handle = ctx.part_element_world_aabb(door, elem="door_handle")
    with ctx.pose({door_joint: 0.5}):
        open_handle = ctx.part_element_world_aabb(door, elem="door_handle")
    ctx.check(
        "door opens outward from tower wall",
        closed_handle is not None
        and open_handle is not None
        and 0.5 * (open_handle[0][0] + open_handle[1][0])
        > 0.5 * (closed_handle[0][0] + closed_handle[1][0]) + 0.12,
        details=f"closed_handle={closed_handle}, open_handle={open_handle}",
    )

    service_box_rest = ctx.part_element_world_aabb(lantern, elem="rear_service_box")
    with ctx.pose({lantern_joint: math.pi / 2.0}):
        service_box_quarter_turn = ctx.part_element_world_aabb(lantern, elem="rear_service_box")
    ctx.check(
        "lantern housing actually rotates around pedestal axis",
        service_box_rest is not None
        and service_box_quarter_turn is not None
        and abs(
            0.5 * (service_box_quarter_turn[0][1] + service_box_quarter_turn[1][1])
            - 0.5 * (service_box_rest[0][1] + service_box_rest[1][1])
        )
        > 0.10,
        details=f"rest={service_box_rest}, quarter_turn={service_box_quarter_turn}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
