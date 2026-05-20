from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    Material,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


def _tube_mesh(outer_radius: float, inner_radius: float, length: float, name: str):
    tube = cq.Workplane("XY").circle(outer_radius).circle(inner_radius).extrude(length)
    return mesh_from_cadquery(tube, name, tolerance=0.0006, angular_tolerance=0.08)


def _radial_origin(radius: float, angle: float, z: float, *, yaw_offset: float = 0.0) -> Origin:
    return Origin(
        xyz=(radius * math.cos(angle), radius * math.sin(angle), z),
        rpy=(0.0, 0.0, angle + yaw_offset),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="portable_tripod_device")

    satin_black = Material("satin_black", rgba=(0.02, 0.022, 0.024, 1.0))
    dark_graphite = Material("dark_graphite", rgba=(0.12, 0.13, 0.14, 1.0))
    anodized = Material("dark_anodized_aluminum", rgba=(0.22, 0.24, 0.25, 1.0))
    warm_metal = Material("brushed_aluminum", rgba=(0.58, 0.60, 0.56, 1.0))
    rubber = Material("matte_rubber", rgba=(0.015, 0.014, 0.013, 1.0))
    lens_glass = Material("smoked_lens_glass", rgba=(0.02, 0.035, 0.06, 1.0))
    device_shell = Material("device_shell", rgba=(0.09, 0.11, 0.13, 1.0))

    crown = model.part("crown")
    crown.visual(
        Cylinder(radius=0.055, length=0.045),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=dark_graphite,
        name="lower_collar",
    )
    crown.visual(
        _tube_mesh(0.030, 0.018, 0.270, "hollow_center_sleeve"),
        origin=Origin(xyz=(0.0, 0.0, 0.018)),
        material=anodized,
        name="outer_sleeve",
    )
    crown.visual(
        _tube_mesh(0.039, 0.019, 0.014, "hollow_sleeve_lip"),
        origin=Origin(xyz=(0.0, 0.0, 0.281)),
        material=warm_metal,
        name="sleeve_lip",
    )
    for index, angle in enumerate((0.0, math.pi / 2.0, math.pi, 3.0 * math.pi / 2.0)):
        crown.visual(
            Box((0.0175, 0.0080, 0.0260)),
            origin=_radial_origin(0.02175, angle, 0.170),
            material=anodized,
            name=f"guide_pad_{index}",
        )

    leg_angles = (math.radians(90.0), math.radians(210.0), math.radians(330.0))
    hinge_radius = 0.073
    hinge_z = -0.018
    hinge_gap = 0.048
    for index, angle in enumerate(leg_angles):
        # Crown-side clevis cheeks and a visible steel hinge pin.  Each clevis is
        # oriented with local +X radial and local +Y along the hinge axis.
        offset_x = hinge_radius * math.cos(angle) + (hinge_gap / 2.0) * math.cos(angle + math.pi / 2.0)
        offset_y = hinge_radius * math.sin(angle) + (hinge_gap / 2.0) * math.sin(angle + math.pi / 2.0)
        crown.visual(
            Box((0.050, 0.007, 0.040)),
            origin=Origin(xyz=(offset_x, offset_y, hinge_z), rpy=(0.0, 0.0, angle)),
            material=dark_graphite,
            name=f"hinge_cheek_{index}_b",
        )
        offset_x = hinge_radius * math.cos(angle) - (hinge_gap / 2.0) * math.cos(angle + math.pi / 2.0)
        offset_y = hinge_radius * math.sin(angle) - (hinge_gap / 2.0) * math.sin(angle + math.pi / 2.0)
        crown.visual(
            Box((0.050, 0.007, 0.040)),
            origin=Origin(xyz=(offset_x, offset_y, hinge_z), rpy=(0.0, 0.0, angle)),
            material=dark_graphite,
            name=f"hinge_cheek_{index}_c",
        )
        crown.visual(
            Cylinder(radius=0.0045, length=0.066),
            origin=Origin(
                xyz=(hinge_radius * math.cos(angle), hinge_radius * math.sin(angle), hinge_z),
                rpy=(-math.pi / 2.0, 0.0, angle),
            ),
            material=warm_metal,
            name=f"hinge_pin_{index}",
        )

    leg_length = 0.585
    leg_start = 0.016
    leg_splay = math.radians(24.0)
    leg_dir = (
        math.sin(leg_splay),
        0.0,
        -math.cos(leg_splay),
    )
    for index, angle in enumerate(leg_angles):
        leg = model.part(f"leg_{index}")
        leg.visual(
            Cylinder(radius=0.011, length=0.038),
            origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=anodized,
            name="hinge_eye",
        )
        leg.visual(
            Box((0.024, 0.018, 0.010)),
            origin=Origin(xyz=(0.007, 0.0, -0.013)),
            material=anodized,
            name="hinge_neck",
        )
        leg.visual(
            Cylinder(radius=0.0105, length=leg_length),
            origin=Origin(
                xyz=(leg_dir[0] * (leg_start + leg_length * 0.5), 0.0, leg_dir[2] * (leg_start + leg_length * 0.5)),
                rpy=(0.0, math.pi - leg_splay, 0.0),
            ),
            material=satin_black,
            name="leg_tube",
        )
        leg.visual(
            Cylinder(radius=0.017, length=0.040),
            origin=Origin(
                xyz=(leg_dir[0] * (leg_start + leg_length + 0.010), 0.0, leg_dir[2] * (leg_start + leg_length + 0.010)),
                rpy=(0.0, math.pi - leg_splay, 0.0),
            ),
            material=rubber,
            name="rubber_foot",
        )
        leg.visual(
            Sphere(radius=0.013),
            origin=Origin(xyz=(leg_dir[0] * (leg_start + leg_length), 0.0, leg_dir[2] * (leg_start + leg_length))),
            material=rubber,
            name="foot_end",
        )
        model.articulation(
            f"crown_to_leg_{index}",
            ArticulationType.REVOLUTE,
            parent=crown,
            child=leg,
            origin=Origin(
                xyz=(hinge_radius * math.cos(angle), hinge_radius * math.sin(angle), hinge_z),
                rpy=(0.0, 0.0, angle),
            ),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=12.0, velocity=1.5, lower=-2.10, upper=0.35),
        )

    center_column = model.part("center_column")
    center_column.visual(
        Cylinder(radius=0.014, length=0.620),
        origin=Origin(xyz=(0.0, 0.0, 0.070)),
        material=warm_metal,
        name="inner_column",
    )
    center_column.visual(
        Cylinder(radius=0.026, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.390)),
        material=anodized,
        name="top_flange",
    )
    model.articulation(
        "crown_to_center_column",
        ArticulationType.PRISMATIC,
        parent=crown,
        child=center_column,
        origin=Origin(xyz=(0.0, 0.0, 0.288)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.18, lower=0.0, upper=0.160),
    )

    pan_head = model.part("pan_head")
    pan_head.visual(
        Cylinder(radius=0.039, length=0.026),
        origin=Origin(xyz=(0.0, 0.0, 0.013)),
        material=dark_graphite,
        name="pan_disk",
    )
    pan_head.visual(
        Cylinder(radius=0.017, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, 0.055)),
        material=anodized,
        name="pan_post",
    )
    pan_head.visual(
        Box((0.090, 0.118, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.086)),
        material=dark_graphite,
        name="tilt_bridge",
    )
    pan_head.visual(
        Box((0.060, 0.010, 0.075)),
        origin=Origin(xyz=(0.0, -0.057, 0.121)),
        material=dark_graphite,
        name="tilt_cheek_0",
    )
    pan_head.visual(
        Box((0.060, 0.010, 0.075)),
        origin=Origin(xyz=(0.0, 0.057, 0.121)),
        material=dark_graphite,
        name="tilt_cheek_1",
    )
    model.articulation(
        "center_column_to_pan_head",
        ArticulationType.CONTINUOUS,
        parent=center_column,
        child=pan_head,
        origin=Origin(xyz=(0.0, 0.0, 0.400)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=4.0, velocity=2.5),
    )

    tilt_plate = model.part("tilt_plate")
    tilt_plate.visual(
        Cylinder(radius=0.010, length=0.104),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=warm_metal,
        name="tilt_trunnion",
    )
    tilt_plate.visual(
        Box((0.036, 0.048, 0.021)),
        origin=Origin(xyz=(0.0, 0.0, 0.0105)),
        material=anodized,
        name="trunnion_block",
    )
    tilt_plate.visual(
        Box((0.112, 0.090, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=anodized,
        name="top_plate",
    )
    tilt_plate.visual(
        Box((0.096, 0.073, 0.046)),
        origin=Origin(xyz=(0.004, 0.0, 0.054)),
        material=device_shell,
        name="device_body",
    )
    tilt_plate.visual(
        Cylinder(radius=0.020, length=0.026),
        origin=Origin(xyz=(0.064, 0.0, 0.054), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=lens_glass,
        name="front_lens",
    )
    tilt_plate.visual(
        Box((0.070, 0.020, 0.006)),
        origin=Origin(xyz=(-0.012, 0.0, 0.080)),
        material=satin_black,
        name="top_rail",
    )
    model.articulation(
        "pan_head_to_tilt_plate",
        ArticulationType.REVOLUTE,
        parent=pan_head,
        child=tilt_plate,
        origin=Origin(xyz=(0.0, 0.0, 0.121)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=7.0, velocity=1.4, lower=-0.70, upper=0.90),
    )

    lock_knob = model.part("lock_knob")
    lock_knob.visual(
        Cylinder(radius=0.0042, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=warm_metal,
        name="threaded_shaft",
    )
    for i, z in enumerate((0.004, 0.009, 0.014)):
        lock_knob.visual(
            Cylinder(radius=0.0049, length=0.0014),
            origin=Origin(xyz=(0.0, 0.0, z)),
            material=anodized,
            name=f"thread_crest_{i}",
        )
    lock_knob.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.034,
                0.018,
                body_style="lobed",
                top_diameter=0.030,
                crown_radius=0.001,
                grip=KnobGrip(style="ribbed", count=18, depth=0.0010, width=0.0015),
                center=False,
            ),
            "lobed_lock_knob",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.018)),
        material=satin_black,
        name="knob_cap",
    )
    model.articulation(
        "tilt_plate_to_lock_knob",
        ArticulationType.CONTINUOUS,
        parent=tilt_plate,
        child=lock_knob,
        origin=Origin(xyz=(-0.012, 0.0, 0.083)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.6, velocity=5.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    crown = object_model.get_part("crown")
    center_column = object_model.get_part("center_column")
    pan_head = object_model.get_part("pan_head")
    tilt_plate = object_model.get_part("tilt_plate")
    lock_knob = object_model.get_part("lock_knob")

    column_slide = object_model.get_articulation("crown_to_center_column")

    for index in range(3):
        leg = object_model.get_part(f"leg_{index}")
        ctx.allow_overlap(
            crown,
            leg,
            elem_a=f"hinge_pin_{index}",
            elem_b="hinge_eye",
            reason="The crown hinge pin is intentionally captured through the leg hinge eye.",
        )
        ctx.expect_overlap(
            crown,
            leg,
            axes="xyz",
            elem_a=f"hinge_pin_{index}",
            elem_b="hinge_eye",
            min_overlap=0.006,
            name=f"leg {index} hinge pin is captured in the eye",
        )

    ctx.expect_within(
        center_column,
        crown,
        axes="xy",
        inner_elem="inner_column",
        outer_elem="outer_sleeve",
        margin=0.002,
        name="center column stays inside sleeve bore",
    )
    ctx.expect_overlap(
        center_column,
        crown,
        axes="z",
        elem_a="inner_column",
        elem_b="outer_sleeve",
        min_overlap=0.150,
        name="collapsed column remains deeply inserted",
    )
    rest_column_pos = ctx.part_world_position(center_column)
    with ctx.pose({column_slide: 0.160}):
        ctx.expect_within(
            center_column,
            crown,
            axes="xy",
            inner_elem="inner_column",
            outer_elem="outer_sleeve",
            margin=0.002,
            name="extended column stays coaxial in sleeve",
        )
        ctx.expect_overlap(
            center_column,
            crown,
            axes="z",
            elem_a="inner_column",
            elem_b="outer_sleeve",
            min_overlap=0.070,
            name="extended column retains insertion",
        )
        extended_column_pos = ctx.part_world_position(center_column)
    ctx.check(
        "center column slides upward",
        rest_column_pos is not None
        and extended_column_pos is not None
        and extended_column_pos[2] > rest_column_pos[2] + 0.150,
        details=f"rest={rest_column_pos}, extended={extended_column_pos}",
    )

    ctx.expect_contact(
        tilt_plate,
        pan_head,
        elem_a="tilt_trunnion",
        elem_b="tilt_cheek_0",
        contact_tol=0.0005,
        name="tilt trunnion contacts one yoke cheek",
    )
    ctx.expect_contact(
        tilt_plate,
        pan_head,
        elem_a="tilt_trunnion",
        elem_b="tilt_cheek_1",
        contact_tol=0.0005,
        name="tilt trunnion contacts opposite yoke cheek",
    )
    ctx.expect_contact(
        lock_knob,
        tilt_plate,
        elem_a="threaded_shaft",
        elem_b="top_rail",
        contact_tol=0.0005,
        name="locking knob shaft seats on the top rail",
    )

    return ctx.report()


object_model = build_object_model()
