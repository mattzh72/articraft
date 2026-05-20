from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeWithHolesGeometry,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    Material,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="document_wall_safe")

    gunmetal = Material("powder_coated_gunmetal", rgba=(0.13, 0.15, 0.16, 1.0))
    dark = Material("black_shadow", rgba=(0.015, 0.016, 0.018, 1.0))
    edge = Material("worn_dark_edge", rgba=(0.07, 0.075, 0.08, 1.0))
    steel = Material("brushed_steel", rgba=(0.58, 0.58, 0.54, 1.0))
    brass = Material("aged_brass", rgba=(0.78, 0.58, 0.25, 1.0))
    white = Material("engraved_white", rgba=(0.94, 0.92, 0.84, 1.0))

    safe_w = 0.56
    safe_h = 0.92
    safe_d = 0.21
    wall = 0.035
    frame_depth = 0.030

    door_w = 0.45
    door_h = 0.80
    door_t = 0.032
    door_y = -0.146
    hinge_x = -door_w / 2.0

    deposit_x = 0.275
    deposit_z = 0.235
    deposit_w = 0.285
    deposit_h = 0.105
    flap_w = 0.335
    flap_h = 0.140
    flap_t = 0.012
    flap_hinge_z = deposit_z + flap_h / 2.0

    body = model.part("body")
    body.meta["front_axis"] = "-y"
    body.meta["document_scale_m"] = {"width": safe_w, "height": safe_h, "depth": safe_d}

    # Hollow steel wall-safe carcass: back, sides, top/bottom, and a real front
    # frame around the tall rectangular opening.
    body.visual(
        Box((safe_w, 0.026, safe_h)),
        origin=Origin(xyz=(0.0, safe_d / 2.0 - 0.013, 0.0)),
        material=dark,
        name="back_panel",
    )
    body.visual(
        Box((wall, safe_d, safe_h)),
        origin=Origin(xyz=(-safe_w / 2.0 + wall / 2.0, 0.0, 0.0)),
        material=gunmetal,
        name="side_wall_0",
    )
    body.visual(
        Box((wall, safe_d, safe_h)),
        origin=Origin(xyz=(safe_w / 2.0 - wall / 2.0, 0.0, 0.0)),
        material=gunmetal,
        name="side_wall_1",
    )
    body.visual(
        Box((safe_w, safe_d, wall)),
        origin=Origin(xyz=(0.0, 0.0, safe_h / 2.0 - wall / 2.0)),
        material=gunmetal,
        name="top_wall",
    )
    body.visual(
        Box((safe_w, safe_d, wall)),
        origin=Origin(xyz=(0.0, 0.0, -safe_h / 2.0 + wall / 2.0)),
        material=gunmetal,
        name="bottom_wall",
    )
    body.visual(
        Box((0.060, frame_depth, safe_h - 0.040)),
        origin=Origin(xyz=(-safe_w / 2.0 + 0.030, -safe_d / 2.0 + frame_depth / 2.0, 0.0)),
        material=gunmetal,
        name="front_jamb_0",
    )
    body.visual(
        Box((0.060, frame_depth, safe_h - 0.040)),
        origin=Origin(xyz=(safe_w / 2.0 - 0.030, -safe_d / 2.0 + frame_depth / 2.0, 0.0)),
        material=gunmetal,
        name="front_jamb_1",
    )
    body.visual(
        Box((safe_w, frame_depth, 0.060)),
        origin=Origin(xyz=(0.0, -safe_d / 2.0 + frame_depth / 2.0, safe_h / 2.0 - 0.030)),
        material=gunmetal,
        name="front_header",
    )
    body.visual(
        Box((safe_w, frame_depth, 0.060)),
        origin=Origin(xyz=(0.0, -safe_d / 2.0 + frame_depth / 2.0, -safe_h / 2.0 + 0.030)),
        material=gunmetal,
        name="front_sill",
    )
    body.visual(
        Box((0.040, 0.020, 0.78)),
        origin=Origin(xyz=(hinge_x - 0.028, -safe_d / 2.0 + 0.010, 0.0)),
        material=edge,
        name="body_hinge_leaf",
    )
    for i, z in enumerate((-0.285, 0.0, 0.285)):
        body.visual(
            Box((0.020, 0.085, 0.115)),
            origin=Origin(xyz=(hinge_x - 0.014, door_y + 0.004, z)),
            material=edge,
            name=f"body_hinge_web_{i}",
        )
        body.visual(
            Cylinder(0.010, 0.150),
            origin=Origin(xyz=(hinge_x - 0.014, door_y - 0.028, z)),
            material=steel,
            name=f"body_hinge_knuckle_{i}",
        )

    # Single connected mesh panel with a through-cut high deposit opening.
    outer = [(x + door_w / 2.0, z) for x, z in rounded_rect_profile(door_w, door_h, 0.018)]
    opening = [
        (x + deposit_x, z + deposit_z)
        for x, z in rounded_rect_profile(deposit_w, deposit_h, 0.006)
    ]
    door_geom = ExtrudeWithHolesGeometry(outer, [opening], door_t, center=True)
    door_geom.rotate_x(pi / 2.0)

    door = model.part("door")
    door.meta["deposit_opening_m"] = {
        "center_x": deposit_x,
        "center_z": deposit_z,
        "width": deposit_w,
        "height": deposit_h,
    }
    door.visual(
        mesh_from_geometry(door_geom, "door_panel_with_deposit_opening"),
        material=gunmetal,
        name="door_slab",
    )
    door.visual(
        Box((0.030, 0.013, 0.70)),
        origin=Origin(xyz=(0.014, -door_t / 2.0 - 0.006, 0.0)),
        material=edge,
        name="door_hinge_leaf",
    )
    for i, z in enumerate((-0.150, 0.150)):
        door.visual(
            Box((0.035, 0.014, 0.085)),
            origin=Origin(xyz=(0.000, -door_t / 2.0 - 0.011, z)),
            material=edge,
            name=f"door_hinge_web_{i}",
        )
        door.visual(
            Cylinder(0.010, 0.120),
            origin=Origin(xyz=(-0.014, -door_t / 2.0 - 0.012, z)),
            material=steel,
            name=f"door_hinge_knuckle_{i}",
        )

    dial = model.part("dial")
    dial.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.100,
                0.028,
                body_style="faceted",
                top_diameter=0.084,
                edge_radius=0.0015,
                grip=KnobGrip(style="ribbed", count=32, depth=0.0012, width=0.0020),
                indicator=KnobIndicator(style="line", mode="raised", angle_deg=0.0),
                center=False,
            ),
            "rotary_safe_dial",
        ),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="dial_cap",
    )
    dial.visual(
        Box((0.007, 0.002, 0.038)),
        origin=Origin(xyz=(0.0, -0.029, 0.021)),
        material=white,
        name="dial_pointer",
    )

    handle = model.part("lever")
    handle.visual(
        Cylinder(0.026, 0.026),
        origin=Origin(xyz=(0.0, -0.013, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=brass,
        name="handle_hub",
    )
    handle.visual(
        Box((0.110, 0.018, 0.024)),
        origin=Origin(xyz=(0.058, -0.024, 0.0)),
        material=brass,
        name="handle_arm",
    )
    handle.visual(
        Sphere(0.018),
        origin=Origin(xyz=(0.115, -0.024, 0.0)),
        material=brass,
        name="handle_end",
    )

    flap = model.part("flap")
    flap.visual(
        Box((flap_w, flap_t, flap_h)),
        origin=Origin(xyz=(0.0, -flap_t / 2.0, -flap_h / 2.0)),
        material=gunmetal,
        name="flap_panel",
    )
    flap.visual(
        Cylinder(0.007, flap_w),
        origin=Origin(xyz=(0.0, -flap_t / 2.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=steel,
        name="flap_top_barrel",
    )
    flap.visual(
        Box((0.100, 0.006, 0.012)),
        origin=Origin(xyz=(0.0, -flap_t - 0.003, -flap_h + 0.018)),
        material=steel,
        name="flap_pull_lip",
    )

    model.articulation(
        "door_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(hinge_x, door_y, 0.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=35.0, velocity=1.1, lower=0.0, upper=1.75),
    )
    model.articulation(
        "dial_spin",
        ArticulationType.CONTINUOUS,
        parent=door,
        child=dial,
        origin=Origin(xyz=(0.280, -door_t / 2.0, -0.035)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=4.0),
    )
    model.articulation(
        "lever_spindle",
        ArticulationType.REVOLUTE,
        parent=door,
        child=handle,
        origin=Origin(xyz=(0.280, -door_t / 2.0, -0.165)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.5, lower=-0.95, upper=0.95),
    )
    model.articulation(
        "flap_hinge",
        ArticulationType.REVOLUTE,
        parent=door,
        child=flap,
        origin=Origin(xyz=(deposit_x, -door_t / 2.0, flap_hinge_z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=1.6, lower=0.0, upper=1.05),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    door = object_model.get_part("door")
    dial = object_model.get_part("dial")
    lever = object_model.get_part("lever")
    flap = object_model.get_part("flap")
    door_hinge = object_model.get_articulation("door_hinge")
    dial_spin = object_model.get_articulation("dial_spin")
    lever_spindle = object_model.get_articulation("lever_spindle")
    flap_hinge = object_model.get_articulation("flap_hinge")

    def aabb_center(aabb):
        if aabb is None:
            return None
        return tuple((aabb[0][i] + aabb[1][i]) / 2.0 for i in range(3))

    door_box = ctx.part_world_aabb(door)
    body_box = ctx.part_world_aabb(body)
    ctx.check(
        "document safe is tall and narrow",
        body_box is not None
        and (body_box[1][2] - body_box[0][2]) > 0.85
        and (body_box[1][0] - body_box[0][0]) < 0.70,
        details=f"body_aabb={body_box}",
    )
    ctx.check(
        "door is taller than wide",
        door_box is not None
        and (door_box[1][2] - door_box[0][2]) > 1.5 * (door_box[1][0] - door_box[0][0]),
        details=f"door_aabb={door_box}",
    )
    ctx.check(
        "deposit opening recorded as a real cut through the upper panel",
        door.meta.get("deposit_opening_m", {}).get("height", 1.0) < 0.13
        and door.meta.get("deposit_opening_m", {}).get("width", 0.0) > 0.25
        and door.meta.get("deposit_opening_m", {}).get("center_z", 0.0) > 0.18,
        details=f"deposit_meta={door.meta.get('deposit_opening_m')}",
    )

    ctx.expect_gap(
        body,
        door,
        axis="y",
        min_gap=0.001,
        max_gap=0.035,
        positive_elem="front_header",
        negative_elem="door_slab",
        name="closed door sits just in front of the frame",
    )
    ctx.expect_overlap(
        flap,
        door,
        axes="xz",
        elem_a="flap_panel",
        elem_b="door_slab",
        min_overlap=0.08,
        name="deposit flap covers the upper door cutout",
    )
    ctx.expect_gap(
        door,
        flap,
        axis="y",
        max_gap=0.008,
        max_penetration=0.0,
        positive_elem="door_slab",
        negative_elem="flap_panel",
        name="flap is proud of the door face without penetrating it",
    )
    ctx.expect_gap(
        door,
        dial,
        axis="y",
        max_gap=0.006,
        max_penetration=0.0,
        positive_elem="door_slab",
        negative_elem="dial_cap",
        name="dial is mounted on the door face",
    )
    ctx.expect_gap(
        door,
        lever,
        axis="y",
        max_gap=0.006,
        max_penetration=0.0,
        positive_elem="door_slab",
        negative_elem="handle_hub",
        name="lever spindle is mounted on the door face",
    )

    ctx.check(
        "dial is a continuous rotary control",
        getattr(dial_spin, "articulation_type", None) == ArticulationType.CONTINUOUS
        and tuple(round(v, 3) for v in dial_spin.axis) == (0.0, -1.0, 0.0),
        details=f"type={dial_spin.articulation_type}, axis={dial_spin.axis}",
    )
    ctx.check(
        "lever rotates on a short spindle",
        getattr(lever_spindle, "articulation_type", None) == ArticulationType.REVOLUTE
        and lever_spindle.motion_limits is not None
        and lever_spindle.motion_limits.lower < 0.0
        and lever_spindle.motion_limits.upper > 0.0,
        details=f"type={lever_spindle.articulation_type}, limits={lever_spindle.motion_limits}",
    )
    ctx.check(
        "deposit flap uses a top horizontal hinge",
        getattr(flap_hinge, "articulation_type", None) == ArticulationType.REVOLUTE
        and tuple(round(v, 3) for v in flap_hinge.axis) == (-1.0, 0.0, 0.0),
        details=f"type={flap_hinge.articulation_type}, axis={flap_hinge.axis}",
    )

    closed_door_pos = aabb_center(ctx.part_world_aabb(door))
    with ctx.pose({door_hinge: 1.0}):
        open_door_pos = aabb_center(ctx.part_world_aabb(door))
    ctx.check(
        "main door swings outward on the left hinge",
        closed_door_pos is not None
        and open_door_pos is not None
        and open_door_pos[1] < closed_door_pos[1] - 0.05,
        details=f"closed={closed_door_pos}, open={open_door_pos}",
    )

    closed_flap_pos = aabb_center(ctx.part_world_aabb(flap))
    with ctx.pose({flap_hinge: 0.9}):
        open_flap_pos = aabb_center(ctx.part_world_aabb(flap))
    ctx.check(
        "deposit flap swings outward from its top hinge",
        closed_flap_pos is not None
        and open_flap_pos is not None
        and open_flap_pos[1] < closed_flap_pos[1] - 0.025
        and open_flap_pos[2] > closed_flap_pos[2] + 0.020,
        details=f"closed={closed_flap_pos}, open={open_flap_pos}",
    )

    return ctx.report()


object_model = build_object_model()
