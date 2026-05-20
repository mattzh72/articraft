from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _annular_mesh(outer_radius: float, inner_radius: float, height: float, name: str):
    """Return a closed thin cylindrical sleeve/ring mesh with a real central bore."""
    profile = [
        (outer_radius, 0.0),
        (outer_radius, height),
        (inner_radius, height),
        (inner_radius, 0.0),
    ]
    return mesh_from_geometry(LatheGeometry(profile, segments=64, closed=True), name)


def _radial_origin(radius: float, tangent: float, z: float, yaw: float) -> Origin:
    """Place a local radial/tangential feature around the tripod crown."""
    c = math.cos(yaw)
    s = math.sin(yaw)
    return Origin(
        xyz=(radius * c - tangent * s, radius * s + tangent * c, z),
        rpy=(0.0, 0.0, yaw),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="field_telescope_tripod")

    matte_black = model.material("matte_black", rgba=(0.01, 0.012, 0.012, 1.0))
    graphite = model.material("dark_graphite", rgba=(0.08, 0.085, 0.085, 1.0))
    brushed_metal = model.material("brushed_metal", rgba=(0.55, 0.56, 0.54, 1.0))
    rubber = model.material("black_rubber", rgba=(0.015, 0.014, 0.013, 1.0))
    olive = model.material("olive_telescope_body", rgba=(0.24, 0.30, 0.19, 1.0))
    glass = model.material("dark_coated_glass", rgba=(0.02, 0.05, 0.075, 0.82))

    # Root tripod crown/body: a real bored center sleeve with three clevis-style
    # crown hinge forks for the folding legs.
    body = model.part("tripod_body")
    body.visual(
        _annular_mesh(0.045, 0.031, 0.42, "column_sleeve"),
        origin=Origin(xyz=(0.0, 0.0, 0.58)),
        material=graphite,
        name="column_sleeve",
    )
    body.visual(
        _annular_mesh(0.088, 0.031, 0.060, "lower_clamp_ring"),
        origin=Origin(xyz=(0.0, 0.0, 0.60)),
        material=graphite,
        name="lower_clamp_ring",
    )
    body.visual(
        _annular_mesh(0.150, 0.041, 0.120, "crown_ring"),
        origin=Origin(xyz=(0.0, 0.0, 0.76)),
        material=graphite,
        name="crown_ring",
    )
    body.visual(
        _annular_mesh(0.068, 0.031, 0.060, "top_lock_ring"),
        origin=Origin(xyz=(0.0, 0.0, 0.97)),
        material=brushed_metal,
        name="top_lock_ring",
    )

    leg_angles = (0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)
    hinge_radius = 0.190
    hinge_z = 0.820
    for i, yaw in enumerate(leg_angles):
        # Two body-side fork ears leave a visible central gap for the leg knuckle.
        for side in (-1.0, 1.0):
            body.visual(
                Box((0.080, 0.018, 0.080)),
                origin=_radial_origin(0.180, side * 0.044, hinge_z, yaw),
                material=graphite,
                name=f"leg_{i}_fork_{0 if side < 0 else 1}",
            )

    # Sliding center column: long enough to remain captured in the hollow sleeve
    # when raised, with a broad top plate for the pan head.
    column = model.part("center_column")
    column.visual(
        Cylinder(radius=0.024, length=0.900),
        origin=Origin(xyz=(0.0, 0.0, 0.040)),
        material=brushed_metal,
        name="column_tube",
    )
    column.visual(
        Cylinder(radius=0.032, length=0.045),
        origin=Origin(xyz=(0.0, 0.0, -0.365)),
        material=brushed_metal,
        name="lower_guide_bushing",
    )
    column.visual(
        Cylinder(radius=0.032, length=0.045),
        origin=Origin(xyz=(0.0, 0.0, -0.055)),
        material=brushed_metal,
        name="upper_guide_bushing",
    )
    column.visual(
        Cylinder(radius=0.052, length=0.025),
        origin=Origin(xyz=(0.0, 0.0, 0.5025)),
        material=graphite,
        name="top_plate",
    )
    column_slide = model.articulation(
        "body_to_center_column",
        ArticulationType.PRISMATIC,
        parent=body,
        child=column,
        origin=Origin(xyz=(0.0, 0.0, 1.000)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.20, lower=0.0, upper=0.250),
    )

    # Three splayed folding legs.  Each leg frame is at the crown hinge; the
    # visible strut and foot rotate as one folding member about a tangential pin.
    leg_reach = 0.550
    leg_drop = 0.790
    leg_length = math.sqrt(leg_reach * leg_reach + leg_drop * leg_drop)
    leg_pitch = math.atan2(leg_reach, -leg_drop)
    leg_joints = []
    for i, yaw in enumerate(leg_angles):
        leg = model.part(f"leg_{i}")
        leg.visual(
            Cylinder(radius=0.017, length=0.072),
            origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=brushed_metal,
            name="hinge_knuckle",
        )
        leg.visual(
            Cylinder(radius=0.018, length=leg_length),
            origin=Origin(
                xyz=(leg_reach / 2.0, 0.0, -leg_drop / 2.0),
                rpy=(0.0, leg_pitch, 0.0),
            ),
            material=matte_black,
            name="main_strut",
        )
        leg.visual(
            Cylinder(radius=0.024, length=0.090),
            origin=Origin(
                xyz=(leg_reach * 0.56, 0.0, -leg_drop * 0.56),
                rpy=(0.0, leg_pitch, 0.0),
            ),
            material=graphite,
            name="section_collar",
        )
        leg.visual(
            Box((0.125, 0.078, 0.030)),
            origin=Origin(xyz=(leg_reach, 0.0, -leg_drop - 0.015)),
            material=rubber,
            name="rubber_foot",
        )
        leg_joints.append(
            model.articulation(
                f"body_to_leg_{i}",
                ArticulationType.REVOLUTE,
                parent=body,
                child=leg,
                origin=Origin(
                    xyz=(hinge_radius * math.cos(yaw), hinge_radius * math.sin(yaw), hinge_z),
                    rpy=(0.0, 0.0, yaw),
                ),
                axis=(0.0, 1.0, 0.0),
                motion_limits=MotionLimits(effort=35.0, velocity=1.0, lower=-0.25, upper=0.65),
            )
        )

    # Rotating pan head on top of the sliding column.
    head = model.part("pan_head")
    head.visual(
        Cylinder(radius=0.075, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=graphite,
        name="pan_disk",
    )
    head.visual(
        Cylinder(radius=0.033, length=0.075),
        origin=Origin(xyz=(0.0, 0.0, 0.0775)),
        material=brushed_metal,
        name="neck_post",
    )
    head.visual(
        Box((0.185, 0.135, 0.026)),
        origin=Origin(xyz=(0.0, 0.0, 0.123)),
        material=graphite,
        name="cradle_floor",
    )
    for side in (-1.0, 1.0):
        head.visual(
            Box((0.160, 0.018, 0.180)),
            origin=Origin(xyz=(0.0, side * 0.075, 0.195)),
            material=graphite,
            name=f"cradle_cheek_{0 if side < 0 else 1}",
        )
        head.visual(
            Cylinder(radius=0.026, length=0.010),
            origin=Origin(
                xyz=(0.0, side * 0.089, 0.230),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=brushed_metal,
            name=f"tilt_boss_{0 if side < 0 else 1}",
        )
    pan = model.articulation(
        "column_to_pan_head",
        ArticulationType.CONTINUOUS,
        parent=column,
        child=head,
        origin=Origin(xyz=(0.0, 0.0, 0.515)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=20.0, velocity=1.2),
    )

    # Telescope tube on a horizontal tilt hinge through the cradle.
    telescope = model.part("telescope_body")
    telescope.visual(
        Cylinder(radius=0.055, length=0.660),
        origin=Origin(xyz=(0.045, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=olive,
        name="main_tube",
    )
    telescope.visual(
        Cylinder(radius=0.075, length=0.175),
        origin=Origin(xyz=(0.400, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=olive,
        name="objective_bell",
    )
    telescope.visual(
        Cylinder(radius=0.064, length=0.008),
        origin=Origin(xyz=(0.491, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=glass,
        name="objective_lens",
    )
    telescope.visual(
        Cylinder(radius=0.035, length=0.155),
        origin=Origin(xyz=(-0.345, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=matte_black,
        name="eyepiece_tube",
    )
    telescope.visual(
        Cylinder(radius=0.030, length=0.008),
        origin=Origin(xyz=(-0.425, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=glass,
        name="eyepiece_lens",
    )
    telescope.visual(
        Cylinder(radius=0.018, length=0.134),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brushed_metal,
        name="tilt_trunnion",
    )
    tilt = model.articulation(
        "head_to_telescope",
        ArticulationType.REVOLUTE,
        parent=head,
        child=telescope,
        origin=Origin(xyz=(0.0, 0.0, 0.230)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=0.9, lower=-0.55, upper=0.75),
    )

    # Separate focus knob: a visible user control that rotates on the tube.
    focus = model.part("focus_knob")
    focus.visual(
        Cylinder(radius=0.012, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        material=matte_black,
        name="focus_stem",
    )
    focus.visual(
        Cylinder(radius=0.030, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.035)),
        material=matte_black,
        name="knurled_cap",
    )
    focus_spin = model.articulation(
        "telescope_to_focus_knob",
        ArticulationType.CONTINUOUS,
        parent=telescope,
        child=focus,
        origin=Origin(xyz=(-0.055, 0.0, 0.055)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=4.0),
    )

    model.meta["primary_mechanisms"] = {
        "folding_leg_joints": [j.name for j in leg_joints],
        "center_column_slide": column_slide.name,
        "pan_joint": pan.name,
        "tilt_joint": tilt.name,
        "focus_control": focus_spin.name,
    }
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("tripod_body")
    column = object_model.get_part("center_column")
    head = object_model.get_part("pan_head")
    telescope = object_model.get_part("telescope_body")
    focus = object_model.get_part("focus_knob")

    slide = object_model.get_articulation("body_to_center_column")
    tilt = object_model.get_articulation("head_to_telescope")

    for bushing in ("lower_guide_bushing", "upper_guide_bushing"):
        ctx.allow_overlap(
            body,
            column,
            elem_a="column_sleeve",
            elem_b=bushing,
            reason="The guide bushing is intentionally represented with tiny radial bearing interference inside the sliding sleeve.",
        )
    ctx.allow_overlap(
        body,
        column,
        elem_a="lower_clamp_ring",
        elem_b="lower_guide_bushing",
        reason="The lower guide bushing also bears lightly inside the tripod clamp collar.",
    )
    for i in range(3):
        leg = object_model.get_part(f"leg_{i}")
        for fork in (0, 1):
            ctx.allow_overlap(
                body,
                leg,
                elem_a=f"leg_{i}_fork_{fork}",
                elem_b="hinge_knuckle",
                reason="The folding leg knuckle is locally captured in the crown fork bearing.",
            )
    for cheek in ("cradle_cheek_0", "cradle_cheek_1"):
        ctx.allow_overlap(
            head,
            telescope,
            elem_a=cheek,
            elem_b="tilt_trunnion",
            reason="The telescope trunnion is locally captured in the tilt head cheek bearing.",
        )

    ctx.expect_within(
        column,
        body,
        axes="xy",
        inner_elem="column_tube",
        outer_elem="column_sleeve",
        margin=0.003,
        name="center column is centered in the bored tripod sleeve",
    )
    ctx.expect_overlap(
        column,
        body,
        axes="z",
        elem_a="column_tube",
        elem_b="column_sleeve",
        min_overlap=0.38,
        name="collapsed center column remains deeply inserted",
    )
    for bushing in ("lower_guide_bushing", "upper_guide_bushing"):
        ctx.expect_overlap(
            column,
            body,
            axes="z",
            elem_a=bushing,
            elem_b="column_sleeve",
            min_overlap=0.040,
            name=f"{bushing} is seated inside sleeve",
        )
    rest_column_pos = ctx.part_world_position(column)
    with ctx.pose({slide: 0.250}):
        ctx.expect_within(
            column,
            body,
            axes="xy",
            inner_elem="column_tube",
            outer_elem="column_sleeve",
            margin=0.003,
            name="raised center column stays centered in sleeve",
        )
        ctx.expect_overlap(
            column,
            body,
            axes="z",
            elem_a="column_tube",
            elem_b="column_sleeve",
            min_overlap=0.14,
            name="raised center column retains sleeve insertion",
        )
        raised_column_pos = ctx.part_world_position(column)

    ctx.check(
        "center column slides upward",
        rest_column_pos is not None
        and raised_column_pos is not None
        and raised_column_pos[2] > rest_column_pos[2] + 0.20,
        details=f"rest={rest_column_pos}, raised={raised_column_pos}",
    )

    # The tube is captured between the head cheeks and supported by a visible
    # trunnion while leaving clearance from the cradle floor.
    ctx.expect_within(
        telescope,
        head,
        axes="y",
        inner_elem="tilt_trunnion",
        outer_elem="cradle_floor",
        margin=0.010,
        name="tilt trunnion fits inside cradle width",
    )
    ctx.expect_gap(
        telescope,
        head,
        axis="z",
        positive_elem="main_tube",
        negative_elem="cradle_floor",
        min_gap=0.035,
        name="telescope clears cradle floor at level tilt",
    )

    rest_front = ctx.part_element_world_aabb(telescope, elem="objective_lens")
    with ctx.pose({tilt: 0.70}):
        raised_front = ctx.part_element_world_aabb(telescope, elem="objective_lens")
    ctx.check(
        "positive tilt raises the objective end",
        rest_front is not None
        and raised_front is not None
        and raised_front[0][2] > rest_front[0][2] + 0.20,
        details=f"rest={rest_front}, tilted={raised_front}",
    )

    for i in range(3):
        leg = object_model.get_part(f"leg_{i}")
        ctx.expect_contact(
            leg,
            body,
            elem_a="hinge_knuckle",
            elem_b=f"leg_{i}_fork_0",
            contact_tol=0.010,
            name=f"leg {i} hinge knuckle sits in its fork",
        )

    ctx.expect_contact(
        focus,
        telescope,
        elem_a="focus_stem",
        elem_b="main_tube",
        contact_tol=0.002,
        name="focus knob stem is seated on telescope tube",
    )

    return ctx.report()


object_model = build_object_model()
