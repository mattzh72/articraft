from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    ExtrudeGeometry,
    SlotPatternPanelGeometry,
    tube_from_spline_points,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="superautomatic_espresso_machine")

    model.material("graphite", rgba=(0.18, 0.19, 0.21, 1.0))
    model.material("black", rgba=(0.08, 0.08, 0.09, 1.0))
    model.material("silver", rgba=(0.70, 0.71, 0.73, 1.0))
    model.material("steel", rgba=(0.76, 0.77, 0.79, 1.0))
    model.material("tank_smoke", rgba=(0.36, 0.40, 0.45, 0.55))

    body_w = 0.29
    body_d = 0.42
    body_h = 0.47
    wall_t = 0.008

    body = model.part("body")

    # Lower chassis with an open central tray tunnel.
    body.visual(
        Box((0.27, body_w, 0.02)),
        origin=Origin(xyz=(-0.075, 0.0, 0.01)),
        material="graphite",
        name="rear_base",
    )
    body.visual(
        Box((0.15, 0.045, 0.05)),
        origin=Origin(xyz=(0.135, -0.1225, 0.025)),
        material="graphite",
        name="left_guide",
    )
    body.visual(
        Box((0.15, 0.045, 0.05)),
        origin=Origin(xyz=(0.135, 0.1225, 0.025)),
        material="graphite",
        name="right_guide",
    )

    # Main housing shell.
    body.visual(
        Box((body_d, wall_t, 0.45)),
        origin=Origin(xyz=(0.0, -(body_w - wall_t) / 2, 0.245)),
        material="graphite",
        name="left_wall",
    )
    body.visual(
        Box((0.12, wall_t, 0.45)),
        origin=Origin(xyz=(0.15, (body_w - wall_t) / 2, 0.245)),
        material="graphite",
        name="right_front_pillar",
    )
    body.visual(
        Box((0.08, wall_t, 0.45)),
        origin=Origin(xyz=(-0.17, (body_w - wall_t) / 2, 0.245)),
        material="graphite",
        name="right_rear_pillar",
    )
    body.visual(
        Box((0.22, wall_t, 0.118)),
        origin=Origin(xyz=(-0.02, (body_w - wall_t) / 2, 0.079)),
        material="graphite",
        name="right_lower_strip",
    )
    body.visual(
        Box((0.22, wall_t, 0.146)),
        origin=Origin(xyz=(-0.02, (body_w - wall_t) / 2, 0.397)),
        material="graphite",
        name="right_upper_strip",
    )
    body.visual(
        Box((wall_t, body_w - 2 * wall_t, 0.45)),
        origin=Origin(xyz=(-(body_d - wall_t) / 2, 0.0, 0.245)),
        material="graphite",
        name="back_wall",
    )
    body.visual(
        Box((0.22, body_w, 0.012)),
        origin=Origin(xyz=(0.10, 0.0, body_h - 0.006)),
        material="graphite",
        name="top_roof",
    )

    # Front appliance frame around the cup bay.
    body.visual(
        Box((0.05, 0.09, 0.35)),
        origin=Origin(xyz=(0.185, -0.10, 0.255)),
        material="graphite",
        name="left_stile",
    )
    body.visual(
        Box((0.05, 0.09, 0.35)),
        origin=Origin(xyz=(0.185, 0.10, 0.255)),
        material="graphite",
        name="right_stile",
    )
    body.visual(
        Box((0.05, 0.22, 0.08)),
        origin=Origin(xyz=(0.185, 0.0, 0.155)),
        material="black",
        name="lower_fascia",
    )
    body.visual(
        Box((0.05, 0.22, 0.10)),
        origin=Origin(xyz=(0.185, 0.0, 0.385)),
        material="black",
        name="upper_header",
    )
    body.visual(
        Box((0.10, 0.17, 0.16)),
        origin=Origin(xyz=(0.11, 0.0, 0.255)),
        material="black",
        name="brew_back",
    )
    body.visual(
        Box((0.05, 0.06, 0.10)),
        origin=Origin(xyz=(0.165, 0.0, 0.285)),
        material="silver",
        name="dispenser",
    )
    body.visual(
        Cylinder(radius=0.004, length=0.025),
        origin=Origin(xyz=(0.186, -0.012, 0.2225)),
        material="silver",
        name="spout_0",
    )
    body.visual(
        Cylinder(radius=0.004, length=0.025),
        origin=Origin(xyz=(0.186, 0.012, 0.2225)),
        material="silver",
        name="spout_1",
    )

    # Water-tank surround at the rear top.
    body.visual(
        Box((0.012, 0.17, 0.05)),
        origin=Origin(xyz=(-0.204, 0.0, 0.435)),
        material="tank_smoke",
        name="tank_rear",
    )
    body.visual(
        Box((0.192, 0.012, 0.05)),
        origin=Origin(xyz=(-0.108, -0.079, 0.435)),
        material="tank_smoke",
        name="tank_left",
    )
    body.visual(
        Box((0.192, 0.012, 0.05)),
        origin=Origin(xyz=(-0.108, 0.079, 0.435)),
        material="tank_smoke",
        name="tank_right",
    )

    # Wand support boss on the side wall.
    body.visual(
        Cylinder(radius=0.014, length=0.024),
        origin=Origin(
            xyz=(0.13, -(body_w / 2 + 0.012), 0.32),
            rpy=(pi / 2, 0.0, 0.0),
        ),
        material="silver",
        name="wand_pivot",
    )

    # Water-tank lid.
    lid = model.part("water_lid")
    lid_mesh = mesh_from_geometry(
        ExtrudeGeometry(
            rounded_rect_profile(0.156, 0.146, 0.012),
            0.010,
            cap=True,
            center=True,
        ),
        "water_lid_panel",
    )
    lid.visual(
        lid_mesh,
        origin=Origin(xyz=(0.078, 0.0, 0.003)),
        material="tank_smoke",
        name="panel",
    )
    lid.visual(
        Box((0.018, 0.038, 0.010)),
        origin=Origin(xyz=(0.152, 0.0, 0.008)),
        material="black",
        name="tab",
    )

    # Side service door.
    door = model.part("service_door")
    door.visual(
        Box((0.216, 0.010, 0.186)),
        origin=Origin(xyz=(0.108, 0.0, 0.093)),
        material="black",
        name="panel",
    )
    door.visual(
        Box((0.040, 0.012, 0.028)),
        origin=Origin(xyz=(0.176, 0.0, 0.125)),
        material="silver",
        name="pull",
    )

    # Drip tray with a removable sliding pan and slotted grate.
    tray = model.part("drip_tray")
    tray_len = 0.18
    tray_w = 0.186
    tray.visual(
        Box((tray_len, tray_w, 0.003)),
        origin=Origin(xyz=(tray_len / 2, 0.0, 0.0015)),
        material="black",
        name="pan_floor",
    )
    tray.visual(
        Box((tray_len, 0.004, 0.028)),
        origin=Origin(xyz=(tray_len / 2, -(tray_w - 0.004) / 2, 0.014)),
        material="black",
        name="pan_left",
    )
    tray.visual(
        Box((tray_len, 0.004, 0.028)),
        origin=Origin(xyz=(tray_len / 2, (tray_w - 0.004) / 2, 0.014)),
        material="black",
        name="pan_right",
    )
    tray.visual(
        Box((0.004, tray_w, 0.028)),
        origin=Origin(xyz=(0.002, 0.0, 0.014)),
        material="black",
        name="pan_back",
    )
    tray.visual(
        Box((0.012, tray_w, 0.020)),
        origin=Origin(xyz=(tray_len - 0.006, 0.0, 0.010)),
        material="black",
        name="pan_front",
    )
    grate_mesh = mesh_from_geometry(
        SlotPatternPanelGeometry(
            (0.164, 0.178),
            0.003,
            slot_size=(0.026, 0.004),
            pitch=(0.034, 0.014),
            frame=0.008,
            corner_radius=0.004,
            slot_angle_deg=0.0,
            stagger=True,
            center=True,
        ),
        "drip_tray_grate",
    )
    tray.visual(
        grate_mesh,
        origin=Origin(xyz=(0.086, 0.0, 0.0295)),
        material="steel",
        name="grate",
    )

    # Rotary selector on a short front-facing shaft.
    selector = model.part("selector")
    selector.visual(
        Cylinder(radius=0.004, length=0.012),
        origin=Origin(
            xyz=(0.006, 0.0, 0.0),
            rpy=(0.0, pi / 2, 0.0),
        ),
        material="silver",
        name="shaft",
    )
    selector.visual(
        Cylinder(radius=0.027, length=0.018),
        origin=Origin(
            xyz=(0.021, 0.0, 0.0),
            rpy=(0.0, pi / 2, 0.0),
        ),
        material="silver",
        name="knob",
    )
    selector.visual(
        Box((0.006, 0.012, 0.008)),
        origin=Origin(xyz=(0.030, 0.0, 0.023)),
        material="black",
        name="mark",
    )

    # Milk wand as a bent tube that swivels around a vertical pivot.
    wand = model.part("milk_wand")
    wand_mesh = mesh_from_geometry(
        tube_from_spline_points(
            [
                (0.0, 0.0, 0.0),
                (0.0, -0.016, -0.003),
                (0.012, -0.034, -0.040),
                (0.022, -0.034, -0.102),
                (0.024, -0.024, -0.172),
            ],
            radius=0.004,
            samples_per_segment=18,
            radial_segments=18,
            cap_ends=True,
        ),
        "milk_wand_tube",
    )
    wand.visual(
        wand_mesh,
        material="silver",
        name="tube",
    )

    model.articulation(
        "body_to_water_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(-0.178, 0.0, 0.460)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=2.0,
            lower=0.0,
            upper=1.30,
        ),
    )
    model.articulation(
        "body_to_service_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(-0.128, (body_w - wall_t) / 2, 0.138)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=2.0,
            lower=0.0,
            upper=1.45,
        ),
    )
    model.articulation(
        "body_to_drip_tray",
        ArticulationType.PRISMATIC,
        parent=body,
        child=tray,
        origin=Origin(xyz=(0.030, 0.0, 0.020)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=45.0,
            velocity=0.20,
            lower=0.0,
            upper=0.11,
        ),
    )
    model.articulation(
        "body_to_selector",
        ArticulationType.REVOLUTE,
        parent=body,
        child=selector,
        origin=Origin(xyz=(0.210, 0.0, 0.165)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=4.0,
            lower=-1.2,
            upper=1.2,
        ),
    )
    model.articulation(
        "body_to_milk_wand",
        ArticulationType.REVOLUTE,
        parent=body,
        child=wand,
        origin=Origin(xyz=(0.130, -body_w / 2 - 0.024, 0.320)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=2.0,
            lower=0.0,
            upper=1.10,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    tray = object_model.get_part("drip_tray")
    door = object_model.get_part("service_door")
    lid = object_model.get_part("water_lid")
    wand = object_model.get_part("milk_wand")
    selector = object_model.get_part("selector")

    tray_slide = object_model.get_articulation("body_to_drip_tray")
    door_hinge = object_model.get_articulation("body_to_service_door")
    lid_hinge = object_model.get_articulation("body_to_water_lid")
    wand_pivot = object_model.get_articulation("body_to_milk_wand")
    selector_joint = object_model.get_articulation("body_to_selector")

    def aabb_center(aabb):
        if aabb is None:
            return None
        return tuple((aabb[0][i] + aabb[1][i]) * 0.5 for i in range(3))

    ctx.expect_origin_gap(
        selector,
        tray,
        axis="z",
        min_gap=0.10,
        name="selector sits above the drip tray",
    )
    ctx.expect_within(
        tray,
        body,
        axes="y",
        margin=0.0,
        name="drip tray stays centered between the side guides",
    )

    tray_rest = ctx.part_world_position(tray)
    with ctx.pose({tray_slide: 0.11}):
        tray_extended = ctx.part_world_position(tray)
        ctx.expect_overlap(
            tray,
            body,
            axes="x",
            min_overlap=0.06,
            name="extended tray keeps retained insertion",
        )
        ctx.expect_within(
            tray,
            body,
            axes="y",
            margin=0.0,
            name="extended tray remains laterally guided",
        )
    ctx.check(
        "drip tray extends forward",
        tray_rest is not None
        and tray_extended is not None
        and tray_extended[0] > tray_rest[0] + 0.09,
        details=f"rest={tray_rest}, extended={tray_extended}",
    )

    door_rest = aabb_center(ctx.part_world_aabb(door))
    with ctx.pose({door_hinge: 1.15}):
        door_open = aabb_center(ctx.part_world_aabb(door))
    ctx.check(
        "service door swings outward",
        door_rest is not None
        and door_open is not None
        and door_open[1] > door_rest[1] + 0.04,
        details=f"rest={door_rest}, open={door_open}",
    )

    lid_rest = aabb_center(ctx.part_world_aabb(lid))
    with ctx.pose({lid_hinge: 1.05}):
        lid_open = aabb_center(ctx.part_world_aabb(lid))
    ctx.check(
        "water lid opens upward",
        lid_rest is not None
        and lid_open is not None
        and lid_open[2] > lid_rest[2] + 0.05,
        details=f"rest={lid_rest}, open={lid_open}",
    )

    wand_rest = aabb_center(ctx.part_world_aabb(wand))
    with ctx.pose({wand_pivot: 1.10}):
        wand_swung = aabb_center(ctx.part_world_aabb(wand))
    ctx.check(
        "milk wand swings forward",
        wand_rest is not None
        and wand_swung is not None
        and wand_swung[0] > wand_rest[0] + 0.010
        and wand_swung[1] > wand_rest[1] + 0.015,
        details=f"rest={wand_rest}, swung={wand_swung}",
    )

    mark_rest = aabb_center(ctx.part_element_world_aabb(selector, elem="mark"))
    with ctx.pose({selector_joint: 1.0}):
        mark_rotated = aabb_center(ctx.part_element_world_aabb(selector, elem="mark"))
    ctx.check(
        "selector marker rotates around the shaft",
        mark_rest is not None
        and mark_rotated is not None
        and abs(mark_rotated[1] - mark_rest[1]) > 0.012
        and abs(mark_rotated[2] - mark_rest[2]) > 0.006,
        details=f"rest={mark_rest}, rotated={mark_rotated}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
