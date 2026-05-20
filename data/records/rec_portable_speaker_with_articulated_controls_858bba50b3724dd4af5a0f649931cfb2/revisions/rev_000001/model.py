from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    KnobTopFeature,
    Material,
    MotionLimits,
    Origin,
    PerforatedPanelGeometry,
    TestContext,
    TestReport,
    TireGeometry,
    TireShoulder,
    TireSidewall,
    TireTread,
    WheelBore,
    WheelFace,
    WheelGeometry,
    WheelHub,
    WheelRim,
    WheelSpokes,
    mesh_from_cadquery,
    mesh_from_geometry,
)
import cadquery as cq


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_karaoke_speaker")

    black = model.material("molded_black", rgba=(0.015, 0.016, 0.018, 1.0))
    charcoal = model.material("charcoal_panel", rgba=(0.055, 0.058, 0.065, 1.0))
    grille_mat = model.material("perforated_grille", rgba=(0.008, 0.009, 0.010, 1.0))
    rubber = model.material("soft_rubber", rgba=(0.006, 0.006, 0.006, 1.0))
    satin = model.material("satin_metal", rgba=(0.62, 0.66, 0.68, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.16, 0.17, 0.18, 1.0))
    blue = model.material("blue_button", rgba=(0.05, 0.25, 0.85, 1.0))
    white = model.material("white_mark", rgba=(0.85, 0.88, 0.90, 1.0))

    width = 0.34
    depth = 0.24
    height = 0.46

    body = model.part("body")

    shell_shape = (
        cq.Workplane("XY")
        .box(width, depth, height)
        .edges("|Z")
        .fillet(0.035)
        .edges(">Z")
        .fillet(0.012)
    )
    body.visual(
        mesh_from_cadquery(shell_shape, "rounded_speaker_shell", tolerance=0.0015),
        origin=Origin(xyz=(0.0, 0.0, height * 0.5)),
        material=black,
        name="rounded_shell",
    )

    body.visual(
        Box((0.285, 0.170, 0.012)),
        origin=Origin(xyz=(0.0, -0.020, height + 0.006)),
        material=charcoal,
        name="top_control_deck",
    )
    body.visual(
        Box((0.275, 0.008, 0.080)),
        origin=Origin(xyz=(0.0, -depth * 0.5 - 0.004, 0.385)),
        material=charcoal,
        name="front_control_face",
    )

    grille = PerforatedPanelGeometry(
        (0.270, 0.275),
        0.004,
        hole_diameter=0.007,
        pitch=(0.014, 0.014),
        frame=0.014,
        corner_radius=0.018,
        stagger=True,
    )
    body.visual(
        mesh_from_geometry(grille, "front_perforated_grille"),
        origin=Origin(
            xyz=(0.0, -depth * 0.5 - 0.002, 0.215),
            rpy=(-math.pi / 2.0, 0.0, 0.0),
        ),
        material=grille_mat,
        name="front_grille",
    )

    # A shallow colored rim and two driver shadows give the front face the scale
    # and read of a portable karaoke speaker rather than a plain box.
    body.visual(
        Cylinder(radius=0.122, length=0.004),
        origin=Origin(
            xyz=(0.0, -depth * 0.5 - 0.0045, 0.220),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=dark_metal,
        name="woofer_bezel",
    )
    body.visual(
        Cylinder(radius=0.082, length=0.003),
        origin=Origin(
            xyz=(0.0, -depth * 0.5 - 0.0075, 0.220),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=black,
        name="woofer_shadow",
    )
    body.visual(
        Cylinder(radius=0.032, length=0.004),
        origin=Origin(
            xyz=(0.0, -depth * 0.5 - 0.0045, 0.335),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=dark_metal,
        name="tweeter_bezel",
    )

    # Rear trolley guide channels.  They sit behind the rear shell, open around
    # the moving chrome rails, and are tied back to the shell by small bosses.
    for side_index, x in enumerate((-0.105, 0.105)):
        body.visual(
            Box((0.048, 0.016, 0.016)),
            origin=Origin(xyz=(x, depth * 0.5 + 0.008, 0.115)),
            material=dark_metal,
            name=f"guide_lower_boss_{side_index}",
        )
        body.visual(
            Box((0.048, 0.016, 0.016)),
            origin=Origin(xyz=(x, depth * 0.5 + 0.008, 0.420)),
            material=dark_metal,
            name=f"guide_upper_boss_{side_index}",
        )
        for side_offset, side_name in ((-0.016, "a"), (0.016, "b")):
            body.visual(
                Box((0.008, 0.028, 0.018)),
                origin=Origin(xyz=(x + side_offset, depth * 0.5 + 0.022, 0.115)),
                material=dark_metal,
                name=f"guide_lower_lug_{side_index}_{side_name}",
            )
            body.visual(
                Box((0.008, 0.028, 0.018)),
                origin=Origin(xyz=(x + side_offset, depth * 0.5 + 0.022, 0.420)),
                material=dark_metal,
                name=f"guide_upper_lug_{side_index}_{side_name}",
            )
            body.visual(
                Box((0.008, 0.026, 0.330)),
                origin=Origin(xyz=(x + side_offset, depth * 0.5 + 0.038, 0.265)),
                material=satin,
                name=f"guide_side_{side_index}_{side_name}",
            )
        body.visual(
            Box((0.038, 0.008, 0.330)),
            origin=Origin(xyz=(x, depth * 0.5 + 0.046, 0.265)),
            material=satin,
            name=f"guide_back_{side_index}",
        )

    body.visual(
        Cylinder(radius=0.010, length=0.390),
        origin=Origin(
            xyz=(0.0, depth * 0.5 + 0.060, 0.060),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=satin,
        name="rear_axle",
    )
    body.visual(
        Box((0.030, 0.120, 0.018)),
        origin=Origin(xyz=(-0.060, depth * 0.5 + 0.030, 0.060)),
        material=dark_metal,
        name="axle_carrier_0",
    )
    body.visual(
        Box((0.030, 0.120, 0.018)),
        origin=Origin(xyz=(0.060, depth * 0.5 + 0.030, 0.060)),
        material=dark_metal,
        name="axle_carrier_1",
    )

    handle = model.part("handle")
    for x in (-0.105, 0.105):
        handle.visual(
            Cylinder(radius=0.008, length=0.480),
            origin=Origin(xyz=(x, 0.0, 0.060)),
            material=satin,
            name=f"rail_{0 if x < 0 else 1}",
        )
    handle.visual(
        Cylinder(radius=0.018, length=0.270),
        origin=Origin(xyz=(0.0, 0.0, 0.318), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rubber,
        name="top_grip",
    )
    handle.visual(
        Box((0.210, 0.020, 0.020)),
        origin=Origin(xyz=(0.0, 0.0, 0.300)),
        material=rubber,
        name="grip_bridge",
    )
    handle_joint = model.articulation(
        "body_to_handle",
        ArticulationType.PRISMATIC,
        parent=body,
        child=handle,
        origin=Origin(xyz=(0.0, depth * 0.5 + 0.034, 0.240)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=90.0, velocity=0.35, lower=0.0, upper=0.260),
    )

    knob_shape = KnobGeometry(
        0.078,
        0.036,
        body_style="faceted",
        base_diameter=0.084,
        top_diameter=0.058,
        edge_radius=0.0012,
        grip=KnobGrip(style="ribbed", count=24, depth=0.0013, width=0.002),
        indicator=KnobIndicator(style="line", mode="raised", angle_deg=0.0),
        top_feature=KnobTopFeature(style="top_insert", diameter=0.026, height=0.0014),
    )
    knob = model.part("main_knob")
    knob.visual(
        mesh_from_geometry(knob_shape, "main_volume_knob"),
        origin=Origin(xyz=(0.0, 0.0, 0.018)),
        material=dark_metal,
        name="knob_cap",
    )
    model.articulation(
        "body_to_main_knob",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=knob,
        origin=Origin(xyz=(0.0, 0.035, height + 0.012)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.5, velocity=8.0),
    )

    for i, (x, mat) in enumerate(((-0.065, blue), (0.065, black))):
        button = model.part(f"mode_button_{i}")
        button.visual(
            Box((0.046, 0.026, 0.012)),
            origin=Origin(xyz=(0.0, 0.0, 0.006)),
            material=mat,
            name="button_cap",
        )
        button.visual(
            Box((0.026, 0.003, 0.0020)),
            origin=Origin(xyz=(0.0, 0.0, 0.0123)),
            material=white,
            name="mode_mark",
        )
        model.articulation(
            f"body_to_mode_button_{i}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=button,
            origin=Origin(xyz=(x, -0.052, height + 0.012)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(effort=2.0, velocity=0.08, lower=0.0, upper=0.006),
        )

    front_button_positions = (-0.082, 0.0, 0.082)
    for i, x in enumerate(front_button_positions):
        button = model.part(f"front_button_{i}")
        button.visual(
            Cylinder(radius=0.0175, length=0.012),
            origin=Origin(xyz=(0.0, -0.006, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=blue if i == 1 else dark_metal,
            name="button_cap",
        )
        button.visual(
            Cylinder(radius=0.006, length=0.0018),
            origin=Origin(xyz=(0.0, -0.0129, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=white,
            name="button_icon",
        )
        model.articulation(
            f"body_to_front_button_{i}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=button,
            origin=Origin(xyz=(x, -depth * 0.5 - 0.008, 0.392)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=2.0, velocity=0.08, lower=0.0, upper=0.006),
        )

    tire_shape = TireGeometry(
        0.056,
        0.036,
        inner_radius=0.040,
        tread=TireTread(style="block", depth=0.0035, count=18, land_ratio=0.58),
        sidewall=TireSidewall(style="rounded", bulge=0.04),
        shoulder=TireShoulder(width=0.004, radius=0.0025),
    )
    wheel_shape = WheelGeometry(
        0.042,
        0.038,
        rim=WheelRim(inner_radius=0.028, flange_height=0.004, flange_thickness=0.0025),
        hub=WheelHub(radius=0.014, width=0.032, cap_style="domed"),
        face=WheelFace(dish_depth=0.002, front_inset=0.0015, rear_inset=0.0015),
        spokes=WheelSpokes(style="straight", count=6, thickness=0.0025, window_radius=0.006),
        bore=WheelBore(style="round", diameter=0.020),
    )
    for i, x in enumerate((-0.145, 0.145)):
        wheel = model.part(f"wheel_{i}")
        wheel.visual(mesh_from_geometry(tire_shape, f"wheel_{i}_tire"), material=rubber, name="tire")
        wheel.visual(mesh_from_geometry(wheel_shape, f"wheel_{i}_rim"), material=satin, name="rim")
        model.articulation(
            f"body_to_wheel_{i}",
            ArticulationType.CONTINUOUS,
            parent=body,
            child=wheel,
            origin=Origin(xyz=(x, depth * 0.5 + 0.060, 0.060)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=1.0, velocity=20.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    handle = object_model.get_part("handle")
    handle_joint = object_model.get_articulation("body_to_handle")

    for i in range(2):
        wheel = object_model.get_part(f"wheel_{i}")
        joint = object_model.get_articulation(f"body_to_wheel_{i}")
        ctx.allow_overlap(
            body,
            wheel,
            elem_a="rear_axle",
            elem_b="rim",
            reason="The metal axle is intentionally captured through the wheel hub bore so the transport wheel reads as mounted.",
        )
        ctx.expect_overlap(
            body,
            wheel,
            axes="x",
            elem_a="rear_axle",
            elem_b="rim",
            min_overlap=0.025,
            name=f"wheel_{i} rim is retained on axle",
        )
        ctx.check(
            f"wheel_{i} spins continuously on x axle",
            joint.articulation_type == ArticulationType.CONTINUOUS and tuple(joint.axis) == (1.0, 0.0, 0.0),
            details=f"type={joint.articulation_type}, axis={joint.axis}",
        )

    ctx.expect_overlap(
        handle,
        body,
        axes="z",
        min_overlap=0.20,
        name="collapsed trolley handle remains in rear guides",
    )
    rest_handle_pos = ctx.part_world_position(handle)
    with ctx.pose({handle_joint: 0.260}):
        ctx.expect_overlap(
            handle,
            body,
            axes="z",
            min_overlap=0.050,
            name="extended trolley handle remains retained",
        )
        extended_handle_pos = ctx.part_world_position(handle)
    ctx.check(
        "trolley handle slides upward",
        rest_handle_pos is not None
        and extended_handle_pos is not None
        and extended_handle_pos[2] > rest_handle_pos[2] + 0.24,
        details=f"rest={rest_handle_pos}, extended={extended_handle_pos}",
    )

    knob_joint = object_model.get_articulation("body_to_main_knob")
    ctx.check(
        "main knob is continuous rotary control",
        knob_joint.articulation_type == ArticulationType.CONTINUOUS and tuple(knob_joint.axis) == (0.0, 0.0, 1.0),
        details=f"type={knob_joint.articulation_type}, axis={knob_joint.axis}",
    )

    for i in range(3):
        button = object_model.get_part(f"front_button_{i}")
        joint = object_model.get_articulation(f"body_to_front_button_{i}")
        rest_pos = ctx.part_world_position(button)
        with ctx.pose({joint: 0.006}):
            depressed_pos = ctx.part_world_position(button)
        ctx.check(
            f"front_button_{i} depresses inward",
            joint.articulation_type == ArticulationType.PRISMATIC
            and rest_pos is not None
            and depressed_pos is not None
            and depressed_pos[1] > rest_pos[1] + 0.005,
            details=f"type={joint.articulation_type}, rest={rest_pos}, depressed={depressed_pos}",
        )

    for i in range(2):
        button = object_model.get_part(f"mode_button_{i}")
        joint = object_model.get_articulation(f"body_to_mode_button_{i}")
        rest_pos = ctx.part_world_position(button)
        with ctx.pose({joint: 0.006}):
            depressed_pos = ctx.part_world_position(button)
        ctx.check(
            f"mode_button_{i} depresses downward",
            joint.articulation_type == ArticulationType.PRISMATIC
            and rest_pos is not None
            and depressed_pos is not None
            and depressed_pos[2] < rest_pos[2] - 0.005,
            details=f"type={joint.articulation_type}, rest={rest_pos}, depressed={depressed_pos}",
        )

    return ctx.report()


object_model = build_object_model()
