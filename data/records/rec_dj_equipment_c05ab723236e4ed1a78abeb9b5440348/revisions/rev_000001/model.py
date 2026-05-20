from __future__ import annotations

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    Material,
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)
import cadquery as cq


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="standalone_dj_sampler")

    body_mat = model.material("warm_black_plastic", rgba=(0.035, 0.038, 0.040, 1.0))
    top_mat = model.material("matte_graphite_panel", rgba=(0.075, 0.080, 0.086, 1.0))
    rubber_mat = model.material("soft_black_rubber", rgba=(0.006, 0.006, 0.006, 1.0))
    knob_mat = model.material("anodized_black_knobs", rgba=(0.010, 0.011, 0.012, 1.0))
    collar_mat = model.material("dark_encoder_collars", rgba=(0.018, 0.019, 0.020, 1.0))
    screen_mat = model.material("glowing_blue_lcd", rgba=(0.08, 0.42, 0.72, 1.0))
    glass_mat = model.material("smoked_glass_bezel", rgba=(0.005, 0.010, 0.014, 1.0))
    hinge_mat = model.material("satin_black_hinge", rgba=(0.012, 0.012, 0.013, 1.0))

    housing_w = 0.46
    housing_d = 0.30
    housing_h = 0.075

    housing = model.part("housing")
    housing_shell = (
        cq.Workplane("XY")
        .box(housing_w, housing_d, housing_h, centered=(True, True, False))
        .edges()
        .chamfer(0.004)
    )
    housing.visual(
        mesh_from_cadquery(housing_shell, "chamfered_sampler_housing", tolerance=0.001),
        material=body_mat,
        name="housing_shell",
    )
    housing.visual(
        Box((0.405, 0.235, 0.0022)),
        origin=Origin(xyz=(0.0, -0.010, housing_h + 0.0002)),
        material=top_mat,
        name="recessed_top_panel",
    )

    # Four fixed collars are part of the case; the rotary encoder caps sit on
    # top of them as separate continuously rotating parts.
    knob_positions = (
        (-0.135, -0.060),
        (-0.045, -0.060),
        (-0.135, 0.045),
        (-0.045, 0.045),
    )
    collar_top_z = housing_h + 0.006
    for idx, (x, y) in enumerate(knob_positions):
        housing.visual(
            Cylinder(radius=0.040, length=0.006),
            origin=Origin(xyz=(x, y, housing_h + 0.003)),
            material=collar_mat,
            name=f"encoder_collar_{idx}",
        )

    # Rear bottom-edge hinge support for the LCD display.
    hinge_y = 0.118
    hinge_z = housing_h + 0.013
    for idx, x in enumerate((-0.080, 0.080)):
        housing.visual(
            Box((0.056, 0.030, 0.011)),
            origin=Origin(xyz=(x, hinge_y, housing_h + 0.0055)),
            material=hinge_mat,
            name=f"hinge_pedestal_{idx}",
        )
        housing.visual(
            Cylinder(radius=0.0105, length=0.048),
            origin=Origin(xyz=(x, hinge_y, hinge_z), rpy=(0.0, 1.57079632679, 0.0)),
            material=hinge_mat,
            name=f"hinge_barrel_{idx}",
        )

    # Small rubber feet touch the underside and keep the standalone unit off the table.
    for idx, (x, y) in enumerate(
        (
            (-0.185, -0.115),
            (0.185, -0.115),
            (-0.185, 0.115),
            (0.185, 0.115),
        )
    ):
        housing.visual(
            Cylinder(radius=0.024, length=0.010),
            origin=Origin(xyz=(x, y, -0.005)),
            material=rubber_mat,
            name=f"rubber_foot_{idx}",
        )

    knob_mesh = mesh_from_geometry(
        KnobGeometry(
            0.064,
            0.034,
            body_style="faceted",
            base_diameter=0.066,
            top_diameter=0.052,
            edge_radius=0.0010,
            grip=KnobGrip(style="ribbed", count=24, depth=0.0010, width=0.0018),
            indicator=KnobIndicator(style="dot", mode="raised", angle_deg=35.0),
            center=False,
        ),
        "large_encoder_knob",
    )
    for idx, (x, y) in enumerate(knob_positions):
        knob = model.part(f"encoder_{idx}")
        knob.visual(
            knob_mesh,
            origin=Origin(),
            material=knob_mat,
            name="knob_cap",
        )
        model.articulation(
            f"housing_to_encoder_{idx}",
            ArticulationType.CONTINUOUS,
            parent=housing,
            child=knob,
            origin=Origin(xyz=(x, y, collar_top_z)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=0.8, velocity=18.0),
            motion_properties=MotionProperties(damping=0.02, friction=0.01),
        )

    display = model.part("lcd_panel")
    # The child frame is the bottom hinge line.  The panel body rises from it,
    # so the revolute axis is exactly along the display's bottom edge.
    display.visual(
        Cylinder(radius=0.010, length=0.104),
        origin=Origin(rpy=(0.0, 1.57079632679, 0.0)),
        material=hinge_mat,
        name="center_hinge_barrel",
    )
    display.visual(
        Box((0.260, 0.014, 0.124)),
        origin=Origin(xyz=(0.0, 0.0, 0.078)),
        material=glass_mat,
        name="display_back_panel",
    )
    display.visual(
        Box((0.102, 0.012, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.013)),
        material=hinge_mat,
        name="hinge_bridge",
    )
    display.visual(
        Box((0.226, 0.0020, 0.083)),
        origin=Origin(xyz=(0.0, -0.0072, 0.083)),
        material=screen_mat,
        name="lcd_screen",
    )
    display.visual(
        Box((0.244, 0.0030, 0.010)),
        origin=Origin(xyz=(0.0, -0.0066, 0.132)),
        material=glass_mat,
        name="screen_bezel_top",
    )
    display.visual(
        Box((0.244, 0.0030, 0.010)),
        origin=Origin(xyz=(0.0, -0.0066, 0.034)),
        material=glass_mat,
        name="screen_bezel_bottom",
    )
    display.visual(
        Box((0.010, 0.0030, 0.102)),
        origin=Origin(xyz=(-0.122, -0.0066, 0.083)),
        material=glass_mat,
        name="screen_bezel_side_0",
    )
    display.visual(
        Box((0.010, 0.0030, 0.102)),
        origin=Origin(xyz=(0.122, -0.0066, 0.083)),
        material=glass_mat,
        name="screen_bezel_side_1",
    )
    model.articulation(
        "housing_to_lcd_panel",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=display,
        origin=Origin(xyz=(0.0, hinge_y, hinge_z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=1.2, lower=0.0, upper=0.70),
        motion_properties=MotionProperties(damping=0.08, friction=0.03),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    housing = object_model.get_part("housing")
    display = object_model.get_part("lcd_panel")
    display_hinge = object_model.get_articulation("housing_to_lcd_panel")

    encoder_joints = [
        object_model.get_articulation(f"housing_to_encoder_{idx}") for idx in range(4)
    ]
    ctx.check(
        "four continuous encoders",
        len(encoder_joints) == 4
        and all(j.articulation_type == ArticulationType.CONTINUOUS for j in encoder_joints),
    )
    for idx, joint in enumerate(encoder_joints):
        knob = object_model.get_part(f"encoder_{idx}")
        ctx.expect_gap(
            knob,
            housing,
            axis="z",
            positive_elem="knob_cap",
            negative_elem=f"encoder_collar_{idx}",
            max_gap=0.0015,
            max_penetration=0.0005,
            name=f"encoder {idx} sits on its collar",
        )
        ctx.check(
            f"encoder {idx} rotates about vertical axis",
            tuple(round(v, 6) for v in joint.axis) == (0.0, 0.0, 1.0),
        )

    ctx.check(
        "lcd hinge uses bottom-edge revolute limits",
        display_hinge.articulation_type == ArticulationType.REVOLUTE
        and display_hinge.motion_limits is not None
        and display_hinge.motion_limits.lower == 0.0
        and display_hinge.motion_limits.upper >= 0.65,
    )
    ctx.expect_gap(
        display,
        housing,
        axis="z",
        positive_elem="center_hinge_barrel",
        negative_elem="recessed_top_panel",
        min_gap=0.0,
        max_gap=0.004,
        name="display hinge is seated just above the housing",
    )

    rest_aabb = ctx.part_world_aabb(display)
    with ctx.pose({display_hinge: 0.60}):
        tilted_aabb = ctx.part_world_aabb(display)
    ctx.check(
        "lcd panel tilts rearward when opened",
        rest_aabb is not None
        and tilted_aabb is not None
        and tilted_aabb[1][1] > rest_aabb[1][1] + 0.035,
        details=f"rest={rest_aabb}, tilted={tilted_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
