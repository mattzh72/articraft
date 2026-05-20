from __future__ import annotations

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
    VentGrilleFrame,
    VentGrilleGeometry,
    VentGrilleMounts,
    VentGrilleSlats,
    VentGrilleSleeve,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="commercial_ice_cream_display_freezer")

    white = model.material("white_enamel", rgba=(0.92, 0.95, 0.96, 1.0))
    blue = model.material("brand_blue", rgba=(0.05, 0.24, 0.62, 1.0))
    liner = model.material("cooler_liner", rgba=(0.72, 0.80, 0.84, 1.0))
    aluminum = model.material("brushed_aluminum", rgba=(0.72, 0.74, 0.72, 1.0))
    dark = model.material("dark_cavity", rgba=(0.035, 0.045, 0.050, 1.0))
    rubber = model.material("black_rubber", rgba=(0.015, 0.014, 0.012, 1.0))
    glass = model.material("slightly_blue_glass", rgba=(0.55, 0.82, 0.95, 0.34))
    panel_metal = model.material("service_panel_gray", rgba=(0.64, 0.67, 0.68, 1.0))

    length = 1.85
    depth = 0.78
    wall = 0.052
    body_bottom = 0.140
    body_height = 0.780
    top_z = body_bottom + body_height
    rear_face_y = depth / 2.0

    body = model.part("body")

    # Insulated hollow chest body: four walls and a floor leave the display well open.
    body.visual(
        Box((length, wall, body_height)),
        origin=Origin(xyz=(0.0, -depth / 2.0 + wall / 2.0, body_bottom + body_height / 2.0)),
        material=white,
        name="front_wall",
    )
    body.visual(
        Box((length, wall, body_height)),
        origin=Origin(xyz=(0.0, depth / 2.0 - wall / 2.0, body_bottom + body_height / 2.0)),
        material=white,
        name="rear_wall",
    )
    body.visual(
        Box((wall, depth, body_height)),
        origin=Origin(xyz=(-length / 2.0 + wall / 2.0, 0.0, body_bottom + body_height / 2.0)),
        material=white,
        name="side_wall_0",
    )
    body.visual(
        Box((wall, depth, body_height)),
        origin=Origin(xyz=(length / 2.0 - wall / 2.0, 0.0, body_bottom + body_height / 2.0)),
        material=white,
        name="side_wall_1",
    )
    body.visual(
        Box((length, depth, 0.090)),
        origin=Origin(xyz=(0.0, 0.0, body_bottom + 0.045)),
        material=white,
        name="floor",
    )
    body.visual(
        Box((length - 0.18, depth - 0.18, 0.036)),
        origin=Origin(xyz=(0.0, 0.0, body_bottom + 0.105)),
        material=liner,
        name="inner_tub_floor",
    )
    body.visual(
        Box((length - 0.20, depth - 0.20, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, body_bottom + 0.128)),
        material=dark,
        name="shadowed_well",
    )

    # Thick rim and two long guide channels at the top edges.
    body.visual(
        Box((length, 0.085, 0.036)),
        origin=Origin(xyz=(0.0, -depth / 2.0 + 0.043, top_z + 0.013)),
        material=aluminum,
        name="front_top_rim",
    )
    body.visual(
        Box((length, 0.085, 0.036)),
        origin=Origin(xyz=(0.0, depth / 2.0 - 0.043, top_z + 0.013)),
        material=aluminum,
        name="rear_top_rim",
    )
    body.visual(
        Box((0.085, depth, 0.036)),
        origin=Origin(xyz=(-length / 2.0 + 0.043, 0.0, top_z + 0.013)),
        material=aluminum,
        name="end_top_rim_0",
    )
    body.visual(
        Box((0.085, depth, 0.036)),
        origin=Origin(xyz=(length / 2.0 - 0.043, 0.0, top_z + 0.013)),
        material=aluminum,
        name="end_top_rim_1",
    )

    rail_y = depth / 2.0 - 0.035
    rail_len = length - 0.12
    rail_floor_z = top_z + 0.037
    for prefix, y in (("front", -rail_y), ("rear", rail_y)):
        body.visual(
            Box((rail_len, 0.052, 0.012)),
            origin=Origin(xyz=(0.0, y, rail_floor_z)),
            material=aluminum,
            name=f"{prefix}_rail_floor",
        )
        body.visual(
            Box((rail_len, 0.008, 0.052)),
            origin=Origin(xyz=(0.0, y - 0.020, top_z + 0.069)),
            material=aluminum,
            name=f"{prefix}_rail_outer_lip",
        )
        body.visual(
            Box((rail_len, 0.008, 0.052)),
            origin=Origin(xyz=(0.0, y + 0.020, top_z + 0.069)),
            material=aluminum,
            name=f"{prefix}_rail_inner_lip",
        )

    body.visual(
        Box((length - 0.14, 0.008, 0.170)),
        origin=Origin(xyz=(0.0, -depth / 2.0 - 0.001, body_bottom + 0.410)),
        material=blue,
        name="front_brand_band",
    )
    body.visual(
        Box((0.42, 0.008, 0.105)),
        origin=Origin(xyz=(-0.57, -depth / 2.0 - 0.001, body_bottom + 0.625)),
        material=aluminum,
        name="front_name_plate",
    )

    # Sliding glass-panel lid, with runners captured by the two U-channel rails.
    lid = model.part("lid")
    lid_len = 1.50
    runner_len = lid_len - 0.12
    runner_height = 0.016
    lid_joint_z = rail_floor_z + 0.006 + runner_height / 2.0

    for prefix, y in (("front", -rail_y), ("rear", rail_y)):
        lid.visual(
            Box((runner_len, 0.016, runner_height)),
            origin=Origin(xyz=(0.0, y, 0.0)),
            material=aluminum,
            name=f"{prefix}_runner",
        )
        lid.visual(
            Box((runner_len, 0.014, 0.060)),
            origin=Origin(xyz=(0.0, y, 0.038)),
            material=aluminum,
            name=f"{prefix}_guide_web",
        )
        lid.visual(
            Box((lid_len, 0.065, 0.024)),
            origin=Origin(xyz=(0.0, y, 0.060)),
            material=aluminum,
            name=f"{prefix}_lid_frame",
        )

    lid.visual(
        Box((0.070, 2.0 * rail_y + 0.065, 0.024)),
        origin=Origin(xyz=(-lid_len / 2.0, 0.0, 0.060)),
        material=aluminum,
        name="end_frame_0",
    )
    lid.visual(
        Box((0.070, 2.0 * rail_y + 0.065, 0.024)),
        origin=Origin(xyz=(lid_len / 2.0, 0.0, 0.060)),
        material=aluminum,
        name="end_frame_1",
    )
    lid.visual(
        Box((1.45, 0.650, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.052)),
        material=glass,
        name="glass_panel",
    )
    lid.visual(
        Box((0.55, 0.030, 0.035)),
        origin=Origin(xyz=(0.0, -rail_y - 0.045, 0.077)),
        material=rubber,
        name="front_pull_handle",
    )

    model.articulation(
        "body_to_lid",
        ArticulationType.PRISMATIC,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, 0.0, lid_joint_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=90.0, velocity=0.35, lower=0.0, upper=0.55),
    )

    # Rear compressor service access panel hinged at its right edge.
    panel_width = 0.70
    panel_height = 0.38
    panel_thickness = 0.035
    panel_right_x = 0.79
    panel_bottom_z = body_bottom + 0.150
    panel_hinge_y = rear_face_y + 0.040

    rear_panel = model.part("rear_panel")
    rear_panel.visual(
        Box((panel_width, panel_thickness, panel_height)),
        origin=Origin(xyz=(-panel_width / 2.0, -0.0225, panel_height / 2.0)),
        material=panel_metal,
        name="panel_skin",
    )
    rear_panel.visual(
        Cylinder(radius=0.018, length=panel_height + 0.060),
        origin=Origin(xyz=(0.0, 0.0, panel_height / 2.0)),
        material=aluminum,
        name="hinge_barrel",
    )
    rear_panel.visual(
        Box((0.075, 0.018, 0.055)),
        origin=Origin(xyz=(-panel_width + 0.080, 0.000, panel_height * 0.52)),
        material=rubber,
        name="pull_latch",
    )
    rear_vent = VentGrilleGeometry(
        (0.36, 0.18),
        frame=0.018,
        face_thickness=0.005,
        duct_depth=0.026,
        duct_wall=0.003,
        slat_pitch=0.030,
        slat_width=0.012,
        slat_angle_deg=28.0,
        corner_radius=0.010,
        slats=VentGrilleSlats(profile="airfoil", direction="down", divider_count=1),
        frame_profile=VentGrilleFrame(style="beveled", depth=0.002),
        mounts=VentGrilleMounts(style="holes", inset=0.022, hole_diameter=0.006),
        sleeve=VentGrilleSleeve(style="short", depth=0.018, wall=0.003),
    )
    rear_panel.visual(
        mesh_from_geometry(rear_vent, "rear_vent_grille"),
        origin=Origin(xyz=(-0.42, -0.005, panel_height * 0.58), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark,
        name="vent_grille",
    )

    model.articulation(
        "body_to_rear_panel",
        ArticulationType.REVOLUTE,
        parent=body,
        child=rear_panel,
        origin=Origin(xyz=(panel_right_x, panel_hinge_y, panel_bottom_z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.5, lower=0.0, upper=1.45),
    )

    # Four bolted caster assemblies under the insulated cabinet.
    def add_caster(name: str, x: float, y: float) -> None:
        caster = model.part(name)
        caster.visual(
            Box((0.160, 0.110, 0.012)),
            origin=Origin(xyz=(0.0, 0.0, 0.0)),
            material=aluminum,
            name="mount_plate",
        )
        caster.visual(
            Cylinder(radius=0.022, length=0.055),
            origin=Origin(xyz=(0.0, 0.0, -0.030)),
            material=aluminum,
            name="swivel_stem",
        )
        caster.visual(
            Cylinder(radius=0.038, length=0.018),
            origin=Origin(xyz=(0.0, 0.0, -0.018)),
            material=aluminum,
            name="swivel_cap",
        )
        caster.visual(
            Box((0.074, 0.090, 0.018)),
            origin=Origin(xyz=(0.0, 0.0, -0.052)),
            material=aluminum,
            name="fork_crown",
        )
        caster.visual(
            Box((0.034, 0.010, 0.080)),
            origin=Origin(xyz=(0.0, -0.034, -0.087)),
            material=aluminum,
            name="fork_cheek_0",
        )
        caster.visual(
            Box((0.034, 0.010, 0.080)),
            origin=Origin(xyz=(0.0, 0.034, -0.087)),
            material=aluminum,
            name="fork_cheek_1",
        )
        caster.visual(
            Cylinder(radius=0.055, length=0.042),
            origin=Origin(xyz=(0.0, 0.0, -0.079), rpy=(pi / 2.0, 0.0, 0.0)),
            material=rubber,
            name="rubber_wheel",
        )
        caster.visual(
            Cylinder(radius=0.008, length=0.090),
            origin=Origin(xyz=(0.0, 0.0, -0.079), rpy=(pi / 2.0, 0.0, 0.0)),
            material=aluminum,
            name="axle_pin",
        )
        caster.visual(
            Cylinder(radius=0.026, length=0.048),
            origin=Origin(xyz=(0.0, 0.0, -0.079), rpy=(pi / 2.0, 0.0, 0.0)),
            material=panel_metal,
            name="wheel_hub",
        )
        model.articulation(
            f"body_to_{name}",
            ArticulationType.FIXED,
            parent=body,
            child=caster,
            origin=Origin(xyz=(x, y, body_bottom - 0.006)),
        )

    caster_x = length / 2.0 - 0.18
    caster_y = depth / 2.0 - 0.14
    add_caster("front_caster_0", -caster_x, -caster_y)
    add_caster("front_caster_1", caster_x, -caster_y)
    add_caster("rear_caster_0", -caster_x, caster_y)
    add_caster("rear_caster_1", caster_x, caster_y)

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    rear_panel = object_model.get_part("rear_panel")
    lid_slide = object_model.get_articulation("body_to_lid")
    rear_hinge = object_model.get_articulation("body_to_rear_panel")

    ctx.check(
        "glass lid is a prismatic slider",
        lid_slide.articulation_type == ArticulationType.PRISMATIC
        and lid_slide.motion_limits is not None
        and lid_slide.motion_limits.upper is not None
        and lid_slide.motion_limits.upper >= 0.50,
        details=f"type={lid_slide.articulation_type}, limits={lid_slide.motion_limits}",
    )
    ctx.check(
        "rear access panel is hinged",
        rear_hinge.articulation_type == ArticulationType.REVOLUTE
        and rear_hinge.motion_limits is not None
        and rear_hinge.motion_limits.upper is not None
        and rear_hinge.motion_limits.upper > 1.2,
        details=f"type={rear_hinge.articulation_type}, limits={rear_hinge.motion_limits}",
    )

    ctx.expect_gap(
        lid,
        body,
        axis="z",
        positive_elem="front_runner",
        negative_elem="front_rail_floor",
        max_gap=0.001,
        max_penetration=0.0,
        name="front lid runner bears on guide channel",
    )
    ctx.expect_gap(
        lid,
        body,
        axis="z",
        positive_elem="rear_runner",
        negative_elem="rear_rail_floor",
        max_gap=0.001,
        max_penetration=0.0,
        name="rear lid runner bears on guide channel",
    )
    ctx.expect_within(
        lid,
        body,
        axes="y",
        inner_elem="front_runner",
        outer_elem="front_rail_floor",
        margin=0.002,
        name="front runner is centered in rail channel",
    )
    ctx.expect_within(
        lid,
        body,
        axes="y",
        inner_elem="rear_runner",
        outer_elem="rear_rail_floor",
        margin=0.002,
        name="rear runner is centered in rail channel",
    )

    rest_lid_pos = ctx.part_world_position(lid)
    with ctx.pose({lid_slide: 0.55}):
        ctx.expect_overlap(
            lid,
            body,
            axes="x",
            elem_a="front_runner",
            elem_b="front_rail_floor",
            min_overlap=0.45,
            name="extended lid remains retained in front rail",
        )
        ctx.expect_overlap(
            lid,
            body,
            axes="x",
            elem_a="rear_runner",
            elem_b="rear_rail_floor",
            min_overlap=0.45,
            name="extended lid remains retained in rear rail",
        )
        extended_lid_pos = ctx.part_world_position(lid)

    ctx.check(
        "lid slides lengthwise along chest",
        rest_lid_pos is not None
        and extended_lid_pos is not None
        and extended_lid_pos[0] > rest_lid_pos[0] + 0.50
        and abs(extended_lid_pos[1] - rest_lid_pos[1]) < 0.001,
        details=f"rest={rest_lid_pos}, extended={extended_lid_pos}",
    )

    ctx.expect_gap(
        rear_panel,
        body,
        axis="y",
        positive_elem="panel_skin",
        negative_elem="rear_wall",
        max_gap=0.001,
        max_penetration=0.0001,
        name="closed access panel sits against rear face",
    )
    ctx.expect_overlap(
        rear_panel,
        body,
        axes="xz",
        elem_a="panel_skin",
        elem_b="rear_wall",
        min_overlap=0.25,
        name="access panel covers compressor opening area",
    )

    def axis_center(aabb, axis_index: int) -> float | None:
        if aabb is None:
            return None
        lo, hi = aabb

        def coord(vec, index: int) -> float:
            try:
                return vec[index]
            except TypeError:
                return (vec.x, vec.y, vec.z)[index]

        return 0.5 * (coord(lo, axis_index) + coord(hi, axis_index))

    closed_panel_y = axis_center(ctx.part_element_world_aabb(rear_panel, elem="panel_skin"), 1)
    with ctx.pose({rear_hinge: 1.35}):
        open_panel_y = axis_center(ctx.part_element_world_aabb(rear_panel, elem="panel_skin"), 1)

    ctx.check(
        "rear panel swings outward from right-edge hinge",
        closed_panel_y is not None and open_panel_y is not None and open_panel_y > closed_panel_y + 0.20,
        details=f"closed_y={closed_panel_y}, open_y={open_panel_y}",
    )

    caster_names = ("front_caster_0", "front_caster_1", "rear_caster_0", "rear_caster_1")
    ctx.check(
        "freezer has four bolted casters",
        all(object_model.get_part(name) is not None for name in caster_names),
        details=f"casters={caster_names}",
    )
    for caster_name in caster_names:
        caster = object_model.get_part(caster_name)
        ctx.expect_gap(
            body,
            caster,
            axis="z",
            positive_elem="floor",
            negative_elem="mount_plate",
            max_gap=0.001,
            max_penetration=0.0,
            name=f"{caster_name} mounting plate contacts cabinet floor",
        )

    return ctx.report()


object_model = build_object_model()
