from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeWithHolesGeometry,
    Material,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _rounded_visor_shell() -> object:
    """Mesh shell for the padded visor cushion, in the panel frame.

    The panel frame origin is the primary hinge axis; the body hangs downward
    along local -Z at q=0.  A shallow vanity-mirror recess is cut into the
    cabin-facing +Y side so the trim reads as a framed opening rather than a
    printed-on rectangle.
    """

    width = 0.430
    height = 0.215
    thickness = 0.026
    corner_radius = 0.034
    center_x = 0.205
    center_z = -0.133

    outer = rounded_rect_profile(width, height, corner_radius, corner_segments=10)
    # A true through opening behind the satin vanity trim: this reads cleanly as
    # a framed aperture rather than as graphics printed on a flat cushion.
    mirror_hole = rounded_rect_profile(0.168, 0.074, 0.007, corner_segments=4)
    mirror_hole = [(x + 0.027, y + 0.021) for x, y in mirror_hole]

    shell = ExtrudeWithHolesGeometry(outer, [mirror_hole], thickness, center=True)
    # ExtrudeWithHolesGeometry creates an XY profile with depth along local Z.
    # Rotate it so profile-Y becomes visor vertical Z and depth becomes Y.
    shell.rotate_x(math.pi / 2.0)
    shell.translate(center_x, 0.0, center_z)
    return shell


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="premium_windshield_sun_visor")

    matte_headliner = model.material("matte_warm_headliner", rgba=(0.72, 0.69, 0.62, 1.0))
    padded_fabric = model.material("matte_greige_padded_fabric", rgba=(0.58, 0.55, 0.49, 1.0))
    seam_fabric = model.material("slightly_darker_welt", rgba=(0.39, 0.37, 0.33, 1.0))
    satin_black = model.material("satin_black_hardware", rgba=(0.035, 0.034, 0.032, 1.0))
    satin_metal = model.material("brushed_satin_pin", rgba=(0.64, 0.62, 0.57, 1.0))
    mirror_glass = model.material("smoked_mirror_glass", rgba=(0.10, 0.12, 0.13, 0.88))
    label_mat = model.material("muted_safety_label", rgba=(0.88, 0.76, 0.38, 1.0))

    roof_header = model.part("roof_header")
    roof_header.visual(
        Box((0.545, 0.072, 0.012)),
        origin=Origin(xyz=(0.205, -0.006, 0.026)),
        material=matte_headliner,
        name="headliner_plate",
    )
    roof_header.visual(
        Box((0.088, 0.056, 0.018)),
        origin=Origin(xyz=(0.000, 0.000, 0.014)),
        material=satin_black,
        name="mount_base",
    )
    roof_header.visual(
        Cylinder(radius=0.016, length=0.020),
        origin=Origin(xyz=(0.000, 0.000, -0.002)),
        material=satin_black,
        name="pivot_bushing",
    )
    for x in (-0.030, 0.030):
        roof_header.visual(
            Cylinder(radius=0.0065, length=0.003),
            origin=Origin(xyz=(x, 0.000, 0.0255)),
            material=satin_metal,
            name=f"mount_screw_{0 if x < 0 else 1}",
        )

    # A connected retaining clip at the free end makes the roof interface read
    # like a real vehicle headliner module, not an isolated hinge sample.
    roof_header.visual(
        Box((0.050, 0.050, 0.016)),
        origin=Origin(xyz=(0.405, 0.000, 0.012)),
        material=satin_black,
        name="clip_base",
    )
    for y in (-0.017, 0.017):
        roof_header.visual(
            Box((0.040, 0.010, 0.040)),
            origin=Origin(xyz=(0.405, y, -0.014)),
            material=satin_black,
            name=f"clip_jaw_{0 if y < 0 else 1}",
        )
    roof_header.visual(
        Box((0.030, 0.040, 0.012)),
        origin=Origin(xyz=(0.405, 0.000, -0.032)),
        material=satin_black,
        name="clip_bridge",
    )

    hinge_arm = model.part("hinge_arm")
    hinge_arm.visual(
        Cylinder(radius=0.0056, length=0.055),
        origin=Origin(xyz=(0.000, 0.000, -0.0315)),
        material=satin_metal,
        name="vertical_spindle",
    )
    hinge_arm.visual(
        Cylinder(radius=0.011, length=0.014),
        origin=Origin(xyz=(0.000, 0.000, -0.027)),
        material=satin_black,
        name="lower_collar",
    )
    hinge_arm.visual(
        Sphere(radius=0.012),
        origin=Origin(xyz=(0.000, 0.000, -0.065)),
        material=satin_black,
        name="elbow_knuckle",
    )
    hinge_arm.visual(
        Cylinder(radius=0.0062, length=0.430),
        origin=Origin(xyz=(0.215, 0.000, -0.065), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_metal,
        name="hinge_rod",
    )
    hinge_arm.visual(
        Box((0.045, 0.016, 0.014)),
        origin=Origin(xyz=(0.020, 0.000, -0.065)),
        material=satin_black,
        name="rod_root_fairing",
    )

    visor_panel = model.part("visor_panel")
    visor_panel.visual(
        mesh_from_geometry(_rounded_visor_shell(), "padded_visor_shell"),
        material=padded_fabric,
        name="padded_shell",
    )

    # Captured hinge sleeves and short molded tabs show the primary flip axis.
    sleeve_specs = ((0.052, 0.056), (0.188, 0.074), (0.344, 0.060))
    for idx, (x, length) in enumerate(sleeve_specs):
        visor_panel.visual(
            Cylinder(radius=0.010, length=length),
            origin=Origin(xyz=(x, 0.000, 0.000), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=satin_black,
            name=f"hinge_sleeve_{idx}",
        )
        visor_panel.visual(
            Box((length * 0.74, 0.016, 0.020)),
            origin=Origin(xyz=(x, 0.000, -0.017)),
            material=satin_black,
            name=f"sleeve_tab_{idx}",
        )

    # Raised seam/welt management: restrained, shallow, and seated into the pad.
    visor_panel.visual(
        Box((0.348, 0.0030, 0.004)),
        origin=Origin(xyz=(0.222, 0.0142, -0.043)),
        material=seam_fabric,
        name="upper_seam",
    )
    visor_panel.visual(
        Box((0.330, 0.0030, 0.004)),
        origin=Origin(xyz=(0.225, 0.0142, -0.214)),
        material=seam_fabric,
        name="lower_seam",
    )
    for idx, x in enumerate((0.030, 0.410)):
        visor_panel.visual(
            Box((0.004, 0.0030, 0.145)),
            origin=Origin(xyz=(x, 0.0142, -0.128)),
            material=seam_fabric,
            name=f"side_seam_{idx}",
        )

    # Small stitch bars are intentionally subdued, with consistent spacing.
    for idx in range(13):
        x = 0.072 + idx * 0.025
        visor_panel.visual(
            Box((0.010, 0.0026, 0.0024)),
            origin=Origin(xyz=(x, 0.0142, -0.204)),
            material=matte_headliner,
            name=f"lower_stitch_{idx}",
        )
    for idx in range(6):
        z = -0.064 - idx * 0.025
        visor_panel.visual(
            Box((0.0024, 0.0026, 0.010)),
            origin=Origin(xyz=(0.040, 0.0142, z)),
            material=matte_headliner,
            name=f"side_stitch_{idx}",
        )

    # Cleanly framed vanity mirror recess: satin surround with smoked glass inset.
    visor_panel.visual(
        Box((0.176, 0.0020, 0.086)),
        origin=Origin(xyz=(0.232, 0.0100, -0.112)),
        material=satin_black,
        name="mirror_shadow",
    )
    visor_panel.visual(
        Box((0.150, 0.0022, 0.060)),
        origin=Origin(xyz=(0.232, 0.0140, -0.112)),
        material=mirror_glass,
        name="mirror_glass",
    )
    visor_panel.visual(
        Box((0.176, 0.0032, 0.007)),
        origin=Origin(xyz=(0.232, 0.0142, -0.073)),
        material=satin_metal,
        name="mirror_frame_top",
    )
    visor_panel.visual(
        Box((0.176, 0.0032, 0.007)),
        origin=Origin(xyz=(0.232, 0.0142, -0.151)),
        material=satin_metal,
        name="mirror_frame_bottom",
    )
    for idx, x in enumerate((0.141, 0.323)):
        visor_panel.visual(
            Box((0.007, 0.0032, 0.084)),
            origin=Origin(xyz=(x, 0.0142, -0.112)),
            material=satin_metal,
            name=f"mirror_frame_side_{idx}",
        )
    visor_panel.visual(
        Box((0.105, 0.0020, 0.027)),
        origin=Origin(xyz=(0.314, -0.0130, -0.185)),
        material=label_mat,
        name="rear_warning_label",
    )

    model.articulation(
        "roof_to_arm",
        ArticulationType.REVOLUTE,
        parent=roof_header,
        child=hinge_arm,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=4.0, velocity=1.2, lower=0.0, upper=1.55),
    )
    model.articulation(
        "arm_to_panel",
        ArticulationType.REVOLUTE,
        parent=hinge_arm,
        child=visor_panel,
        origin=Origin(xyz=(0.020, 0.0, -0.065)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=1.5, lower=0.0, upper=1.42),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    roof = object_model.get_part("roof_header")
    arm = object_model.get_part("hinge_arm")
    panel = object_model.get_part("visor_panel")
    side_pivot = object_model.get_articulation("roof_to_arm")
    flip_hinge = object_model.get_articulation("arm_to_panel")

    ctx.allow_overlap(
        roof,
        arm,
        elem_a="pivot_bushing",
        elem_b="vertical_spindle",
        reason="The vertical spindle is intentionally captured inside the roof bushing proxy.",
    )
    ctx.expect_within(
        arm,
        roof,
        axes="xy",
        inner_elem="vertical_spindle",
        outer_elem="pivot_bushing",
        margin=0.001,
        name="spindle centered in roof bushing",
    )
    ctx.expect_overlap(
        arm,
        roof,
        axes="z",
        elem_a="vertical_spindle",
        elem_b="pivot_bushing",
        min_overlap=0.006,
        name="spindle retained through bushing height",
    )

    for idx in range(3):
        sleeve_name = f"hinge_sleeve_{idx}"
        ctx.allow_overlap(
            arm,
            panel,
            elem_a="hinge_rod",
            elem_b=sleeve_name,
            reason="The satin hinge rod intentionally passes through the molded visor sleeve.",
        )
        ctx.expect_within(
            arm,
            panel,
            axes="yz",
            inner_elem="hinge_rod",
            outer_elem=sleeve_name,
            margin=0.001,
            name=f"hinge rod centered in sleeve {idx}",
        )
        ctx.expect_overlap(
            arm,
            panel,
            axes="x",
            elem_a="hinge_rod",
            elem_b=sleeve_name,
            min_overlap=0.040,
            name=f"hinge rod retained in sleeve {idx}",
        )

    rest_aabb = ctx.part_world_aabb(panel)
    with ctx.pose({flip_hinge: 1.20}):
        folded_aabb = ctx.part_world_aabb(panel)
    ctx.check(
        "primary hinge lifts visor panel",
        rest_aabb is not None
        and folded_aabb is not None
        and folded_aabb[0][2] > rest_aabb[0][2] + 0.060,
        details=f"rest_aabb={rest_aabb}, folded_aabb={folded_aabb}",
    )

    rest_pos = ctx.part_world_position(panel)
    with ctx.pose({side_pivot: 1.20}):
        side_pos = ctx.part_world_position(panel)
    ctx.check(
        "secondary pivot swings toward side window",
        rest_pos is not None and side_pos is not None and side_pos[1] > rest_pos[1] + 0.010,
        details=f"rest={rest_pos}, side={side_pos}",
    )

    return ctx.report()


object_model = build_object_model()
