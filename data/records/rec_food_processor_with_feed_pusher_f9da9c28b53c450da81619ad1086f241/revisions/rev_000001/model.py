from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    Cylinder,
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    LatheGeometry,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    superellipse_profile,
)


def _circle_profile(radius: float, *, segments: int = 72, center=(0.0, 0.0)):
    cx, cy = center
    return [
        (
            cx + radius * math.cos(2.0 * math.pi * i / segments),
            cy + radius * math.sin(2.0 * math.pi * i / segments),
        )
        for i in range(segments)
    ]


def _shift_profile(profile, dx: float, dy: float):
    return [(x + dx, y + dy) for x, y in profile]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="squat_prep_food_processor")

    base_plastic = model.material("warm_white_plastic", rgba=(0.86, 0.84, 0.78, 1.0))
    dark_panel = model.material("charcoal_control_panel", rgba=(0.045, 0.047, 0.050, 1.0))
    clear_bowl = model.material("smoky_clear_bowl", rgba=(0.58, 0.76, 0.92, 0.36))
    clear_lid = model.material("smoky_clear_lid", rgba=(0.64, 0.78, 0.94, 0.42))
    black = model.material("black_rubber", rgba=(0.015, 0.014, 0.013, 1.0))
    steel = model.material("brushed_steel", rgba=(0.74, 0.72, 0.67, 1.0))
    blue_button = model.material("preset_blue", rgba=(0.10, 0.28, 0.58, 1.0))
    white_mark = model.material("white_markings", rgba=(0.96, 0.96, 0.90, 1.0))

    base = model.part("base")
    base_shell = ExtrudeGeometry(
        rounded_rect_profile(0.58, 0.42, 0.070, corner_segments=10),
        0.110,
        center=False,
    )
    base.visual(
        mesh_from_geometry(base_shell, "low_rounded_motor_base"),
        material=base_plastic,
        name="base_shell",
    )
    base.visual(
        Box((0.405, 0.014, 0.074)),
        origin=Origin(xyz=(0.020, -0.214, 0.058)),
        material=dark_panel,
        name="front_control_panel",
    )
    for x in (-0.205, 0.205):
        for y in (-0.145, 0.145):
            base.visual(
                Cylinder(radius=0.030, length=0.010),
                origin=Origin(xyz=(x, y, -0.005)),
                material=black,
                name=f"rubber_foot_{x}_{y}",
            )

    bowl = model.part("bowl")
    bowl_shell = LatheGeometry.from_shell_profiles(
        [
            (0.155, 0.110),
            (0.205, 0.132),
            (0.238, 0.207),
            (0.248, 0.266),
        ],
        [
            (0.118, 0.124),
            (0.170, 0.144),
            (0.207, 0.213),
            (0.222, 0.254),
        ],
        segments=96,
        end_cap="round",
        lip_samples=8,
    )
    bowl.visual(
        mesh_from_geometry(bowl_shell, "wide_hollow_bowl_shell"),
        material=clear_bowl,
        name="bowl_shell",
    )
    bowl.visual(
        Cylinder(radius=0.160, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.116)),
        material=clear_bowl,
        name="bowl_base_ring",
    )
    bowl.visual(
        Cylinder(radius=0.018, length=0.068),
        origin=Origin(xyz=(0.0, 0.0, 0.147)),
        material=clear_bowl,
        name="center_shaft",
    )
    # Fixed rear hinge knuckles mounted to the bowl rim.
    for x in (-0.095, 0.095):
        bowl.visual(
            Cylinder(radius=0.012, length=0.050),
            origin=Origin(xyz=(x, 0.247, 0.290), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=steel,
            name=f"hinge_knuckle_{x}",
        )
        bowl.visual(
            Box((0.060, 0.040, 0.024)),
            origin=Origin(xyz=(x, 0.247, 0.268)),
            material=clear_bowl,
            name=f"hinge_boss_{x}",
        )
    bowl.visual(
        Box((0.090, 0.014, 0.030)),
        origin=Origin(xyz=(0.0, -0.238, 0.252)),
        material=clear_bowl,
        name="front_lock_catch",
    )
    model.articulation(
        "base_to_bowl",
        ArticulationType.FIXED,
        parent=base,
        child=bowl,
        origin=Origin(),
    )

    cutter_disc = model.part("cutter_disc")
    cutter_mesh = ExtrudeWithHolesGeometry(
        _circle_profile(0.126, segments=80),
        [_circle_profile(0.028, segments=48)],
        0.006,
        center=True,
    )
    blade_a = BoxGeometry((0.092, 0.030, 0.007)).translate(0.075, 0.012, 0.003)
    blade_b = BoxGeometry((0.092, 0.030, 0.007)).translate(0.075, -0.012, 0.003).rotate_z(math.pi)
    cutter_mesh.merge(blade_a).merge(blade_b)
    cutter_disc.visual(
        mesh_from_geometry(cutter_mesh, "rotary_cutter_disc"),
        material=steel,
        name="cutter_disc",
    )
    hub_mesh = ExtrudeWithHolesGeometry(
        _circle_profile(0.036, segments=56),
        [_circle_profile(0.021, segments=40)],
        0.026,
        center=False,
    )
    cutter_disc.visual(
        mesh_from_geometry(hub_mesh, "hollow_cutter_disc_hub"),
        material=steel,
        name="disc_hub",
    )
    model.articulation(
        "bowl_to_cutter_disc",
        ArticulationType.CONTINUOUS,
        parent=bowl,
        child=cutter_disc,
        origin=Origin(xyz=(0.0, 0.0, 0.184)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=40.0),
    )

    lid = model.part("lid")
    lid_center_y = -0.245
    feed_y = -0.300
    lid_plate = ExtrudeWithHolesGeometry(
        _circle_profile(0.238, segments=96, center=(0.0, lid_center_y)),
        [_shift_profile(superellipse_profile(0.148, 0.098, exponent=2.35, segments=64), 0.0, feed_y)],
        0.016,
        center=True,
    )
    lid.visual(
        mesh_from_geometry(lid_plate, "clear_lid_plate_with_feed_hole"),
        material=clear_lid,
        name="lid_plate",
    )
    feed_tube = ExtrudeWithHolesGeometry(
        _shift_profile(superellipse_profile(0.160, 0.108, exponent=2.4, segments=64), 0.0, feed_y),
        [_shift_profile(superellipse_profile(0.128, 0.076, exponent=2.4, segments=64), 0.0, feed_y)],
        0.155,
        center=False,
    ).translate(0.0, 0.0, 0.008)
    lid.visual(
        mesh_from_geometry(feed_tube, "broad_oval_feed_tube"),
        material=clear_lid,
        name="feed_tube",
    )
    lid.visual(
        Cylinder(radius=0.012, length=0.092),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="hinge_knuckle",
    )
    lid.visual(
        Box((0.210, 0.040, 0.010)),
        origin=Origin(xyz=(0.0, -0.030, -0.006)),
        material=clear_lid,
        name="hinge_leaf",
    )
    lid.visual(
        Box((0.085, 0.018, 0.022)),
        origin=Origin(xyz=(0.0, -0.492, 0.013)),
        material=clear_lid,
        name="front_lock_tab",
    )
    lid_joint = model.articulation(
        "bowl_to_lid",
        ArticulationType.REVOLUTE,
        parent=bowl,
        child=lid,
        origin=Origin(xyz=(0.0, 0.247, 0.290)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=0.0, upper=1.55),
    )

    pusher = model.part("pusher")
    pusher_plug = ExtrudeGeometry(
        superellipse_profile(0.116, 0.068, exponent=2.4, segments=56),
        0.150,
        center=False,
    ).translate(0.0, 0.0, -0.150)
    pusher_cap = ExtrudeGeometry(
        superellipse_profile(0.178, 0.122, exponent=2.5, segments=64),
        0.030,
        center=False,
    )
    pusher.visual(
        mesh_from_geometry(pusher_plug, "oval_pusher_plug"),
        material=base_plastic,
        name="pusher_plug",
    )
    pusher.visual(
        mesh_from_geometry(pusher_cap, "oval_pusher_cap"),
        material=base_plastic,
        name="pusher_cap",
    )
    pusher.visual(
        Box((0.090, 0.018, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.036)),
        material=base_plastic,
        name="thumb_ridge",
    )
    model.articulation(
        "lid_to_pusher",
        ArticulationType.PRISMATIC,
        parent=lid,
        child=pusher,
        origin=Origin(xyz=(0.0, feed_y, 0.163)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=20.0, velocity=0.35, lower=0.0, upper=0.115),
    )

    front_dial = model.part("front_dial")
    dial_geom = KnobGeometry(
        0.078,
        0.030,
        body_style="skirted",
        top_diameter=0.060,
        edge_radius=0.0015,
        grip=KnobGrip(style="ribbed", count=18, depth=0.0012, width=0.0025),
        indicator=KnobIndicator(style="line", mode="raised", angle_deg=90.0),
        center=False,
    )
    front_dial.visual(
        mesh_from_geometry(dial_geom, "ribbed_front_dial"),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black,
        name="dial_cap",
    )
    front_dial.visual(
        Box((0.006, 0.002, 0.034)),
        origin=Origin(xyz=(0.0, -0.031, 0.016)),
        material=white_mark,
        name="dial_pointer",
    )
    model.articulation(
        "base_to_front_dial",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=front_dial,
        origin=Origin(xyz=(-0.145, -0.221, 0.062)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=0.4, velocity=8.0),
    )

    for i, x in enumerate((0.040, 0.120, 0.200)):
        button = model.part(f"button_{i}")
        button.visual(
            Box((0.052, 0.014, 0.026)),
            origin=Origin(xyz=(0.0, -0.007, 0.0)),
            material=blue_button,
            name="button_cap",
        )
        button.visual(
            Box((0.024, 0.0015, 0.0035)),
            origin=Origin(xyz=(0.0, -0.0142, 0.007)),
            material=white_mark,
            name="preset_mark",
        )
        model.articulation(
            f"base_to_button_{i}",
            ArticulationType.PRISMATIC,
            parent=base,
            child=button,
            origin=Origin(xyz=(x, -0.221, 0.062)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=8.0, velocity=0.12, lower=0.0, upper=0.012),
        )

    model.meta["primary_mechanisms"] = {
        "lid_hinge": lid_joint.name,
        "pusher_slide": "lid_to_pusher",
        "cutter_spin": "bowl_to_cutter_disc",
        "dial_spin": "base_to_front_dial",
        "preset_buttons": ["base_to_button_0", "base_to_button_1", "base_to_button_2"],
    }
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    bowl = object_model.get_part("bowl")
    lid = object_model.get_part("lid")
    pusher = object_model.get_part("pusher")
    cutter = object_model.get_part("cutter_disc")
    lid_joint = object_model.get_articulation("bowl_to_lid")
    pusher_joint = object_model.get_articulation("lid_to_pusher")
    dial_joint = object_model.get_articulation("base_to_front_dial")

    ctx.expect_gap(
        lid,
        bowl,
        axis="z",
        positive_elem="lid_plate",
        negative_elem="bowl_shell",
        min_gap=0.003,
        max_gap=0.025,
        name="closed lid sits just above bowl rim",
    )
    ctx.expect_overlap(
        lid,
        bowl,
        axes="xy",
        elem_a="lid_plate",
        elem_b="bowl_shell",
        min_overlap=0.34,
        name="wide lid covers the bowl footprint",
    )
    ctx.expect_gap(
        lid,
        cutter,
        axis="z",
        positive_elem="lid_plate",
        negative_elem="cutter_disc",
        min_gap=0.075,
        name="visible cavity remains above cutter disc",
    )
    ctx.expect_within(
        cutter,
        bowl,
        axes="xy",
        inner_elem="cutter_disc",
        outer_elem="bowl_shell",
        margin=0.0,
        name="cutter disc stays within the bowl",
    )

    ctx.expect_within(
        pusher,
        lid,
        axes="xy",
        inner_elem="pusher_plug",
        outer_elem="feed_tube",
        margin=0.0,
        name="pusher plug is centered in oval feed tube",
    )
    ctx.expect_overlap(
        pusher,
        lid,
        axes="z",
        elem_a="pusher_plug",
        elem_b="feed_tube",
        min_overlap=0.12,
        name="inserted pusher remains deeply engaged",
    )
    rest_pusher = ctx.part_world_position(pusher)
    with ctx.pose({pusher_joint: 0.100}):
        ctx.expect_overlap(
            pusher,
            lid,
            axes="z",
            elem_a="pusher_plug",
            elem_b="feed_tube",
            min_overlap=0.035,
            name="raised pusher retains insertion in feed tube",
        )
        raised_pusher = ctx.part_world_position(pusher)
    ctx.check(
        "pusher slides upward",
        rest_pusher is not None
        and raised_pusher is not None
        and raised_pusher[2] > rest_pusher[2] + 0.08,
        details=f"rest={rest_pusher}, raised={raised_pusher}",
    )

    closed_lid_box = ctx.part_element_world_aabb(lid, elem="lid_plate")
    with ctx.pose({lid_joint: 1.20}):
        open_lid_box = ctx.part_element_world_aabb(lid, elem="lid_plate")
    closed_lid_z = None if closed_lid_box is None else (closed_lid_box[0][2] + closed_lid_box[1][2]) / 2.0
    open_lid_z = None if open_lid_box is None else (open_lid_box[0][2] + open_lid_box[1][2]) / 2.0
    ctx.check(
        "rear hinge lifts lid upward",
        closed_lid_z is not None and open_lid_z is not None and open_lid_z > closed_lid_z + 0.12,
        details=f"closed_z={closed_lid_z}, open_z={open_lid_z}",
    )

    ctx.check(
        "dial is a continuous rotary control",
        dial_joint.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={dial_joint.articulation_type}",
    )
    ctx.check(
        "cutter disc is a continuous centered rotor",
        object_model.get_articulation("bowl_to_cutter_disc").articulation_type == ArticulationType.CONTINUOUS,
        details="cutter disc should spin about the bowl center shaft",
    )

    for i in range(3):
        button = object_model.get_part(f"button_{i}")
        button_joint = object_model.get_articulation(f"base_to_button_{i}")
        rest = ctx.part_world_position(button)
        with ctx.pose({button_joint: 0.010}):
            pressed = ctx.part_world_position(button)
        ctx.check(
            f"preset button {i} pushes inward",
            rest is not None and pressed is not None and pressed[1] > rest[1] + 0.008,
            details=f"rest={rest}, pressed={pressed}",
        )

    return ctx.report()


object_model = build_object_model()
