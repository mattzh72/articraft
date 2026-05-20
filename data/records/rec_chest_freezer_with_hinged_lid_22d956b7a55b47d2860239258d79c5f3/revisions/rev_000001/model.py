from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="commercial_chest_freezer")

    gloss_white = Material("gloss_white", rgba=(0.94, 0.95, 0.93, 1.0))
    liner_white = Material("cool_liner_white", rgba=(0.86, 0.93, 0.96, 1.0))
    gasket = Material("dark_rubber_gasket", rgba=(0.02, 0.022, 0.024, 1.0))
    metal = Material("brushed_zinc", rgba=(0.62, 0.64, 0.62, 1.0))
    dark = Material("dark_plastic", rgba=(0.05, 0.055, 0.06, 1.0))
    green = Material("green_indicator", rgba=(0.05, 0.75, 0.25, 1.0))
    amber = Material("amber_indicator", rgba=(1.0, 0.62, 0.10, 1.0))

    # A broad commercial chest freezer: width (Y) is much greater than depth (X).
    outer_depth = 0.76
    outer_width = 1.56
    wall = 0.055
    body_top = 0.78
    wall_height = body_top - 0.07
    half_d = outer_depth / 2.0
    half_w = outer_width / 2.0

    body = model.part("body")
    body.visual(
        Box((outer_depth, outer_width, 0.07)),
        origin=Origin(xyz=(0.0, 0.0, 0.035)),
        material=gloss_white,
        name="bottom_pan",
    )
    body.visual(
        Box((wall, outer_width, wall_height)),
        origin=Origin(xyz=(half_d - wall / 2.0, 0.0, 0.07 + wall_height / 2.0)),
        material=gloss_white,
        name="front_wall",
    )
    body.visual(
        Box((wall, outer_width, wall_height)),
        origin=Origin(xyz=(-half_d + wall / 2.0, 0.0, 0.07 + wall_height / 2.0)),
        material=gloss_white,
        name="rear_wall",
    )
    body.visual(
        Box((outer_depth, wall, wall_height)),
        origin=Origin(xyz=(0.0, half_w - wall / 2.0, 0.07 + wall_height / 2.0)),
        material=gloss_white,
        name="side_wall_0",
    )
    body.visual(
        Box((outer_depth, wall, wall_height)),
        origin=Origin(xyz=(0.0, -half_w + wall / 2.0, 0.07 + wall_height / 2.0)),
        material=gloss_white,
        name="side_wall_1",
    )

    # The visible cold well is open at the top rather than modeled as a solid block.
    body.visual(
        Box((0.020, outer_width - 2.0 * wall, 0.58)),
        origin=Origin(xyz=(half_d - wall - 0.010, 0.0, 0.37)),
        material=liner_white,
        name="front_liner",
    )
    body.visual(
        Box((0.020, outer_width - 2.0 * wall, 0.58)),
        origin=Origin(xyz=(-half_d + wall + 0.010, 0.0, 0.37)),
        material=liner_white,
        name="rear_liner",
    )
    body.visual(
        Box((outer_depth - 2.0 * wall, 0.020, 0.58)),
        origin=Origin(xyz=(0.0, half_w - wall - 0.010, 0.37)),
        material=liner_white,
        name="side_liner_0",
    )
    body.visual(
        Box((outer_depth - 2.0 * wall, 0.020, 0.58)),
        origin=Origin(xyz=(0.0, -half_w + wall + 0.010, 0.37)),
        material=liner_white,
        name="side_liner_1",
    )
    body.visual(
        Box((outer_depth - 2.0 * wall, outer_width - 2.0 * wall, 0.025)),
        origin=Origin(xyz=(0.0, 0.0, 0.082)),
        material=liner_white,
        name="well_floor",
    )

    # Raised top lip around the opening.  The lid gasket closes onto this rim.
    body.visual(
        Box((0.095, outer_width + 0.055, 0.035)),
        origin=Origin(xyz=(half_d - 0.025, 0.0, 0.7975)),
        material=gloss_white,
        name="front_top_lip",
    )
    body.visual(
        Box((0.095, outer_width + 0.055, 0.035)),
        origin=Origin(xyz=(-half_d + 0.025, 0.0, 0.7975)),
        material=gloss_white,
        name="rear_top_lip",
    )
    body.visual(
        Box((outer_depth, 0.085, 0.035)),
        origin=Origin(xyz=(0.0, half_w + 0.005, 0.7975)),
        material=gloss_white,
        name="side_top_lip_0",
    )
    body.visual(
        Box((outer_depth, 0.085, 0.035)),
        origin=Origin(xyz=(0.0, -half_w - 0.005, 0.7975)),
        material=gloss_white,
        name="side_top_lip_1",
    )

    # Rear hinge mounting leaves and full-length barrel knuckles.
    hinge_y = 0.42
    hinge_len = 0.58
    for idx, y in enumerate((-hinge_y, hinge_y)):
        body.visual(
            Box((0.075, 0.44, 0.140)),
            origin=Origin(xyz=(-half_d - 0.035, y, 0.735)),
            material=metal,
            name=f"rear_hinge_leaf_{idx}",
        )
        for screw_idx, dz in enumerate((-0.032, 0.032)):
            body.visual(
                Cylinder(radius=0.017, length=0.006),
                origin=Origin(
                    xyz=(-half_d - 0.011, y - 0.16, 0.735 + dz),
                    rpy=(0.0, pi / 2.0, 0.0),
                ),
                material=dark,
                name=f"hinge_screw_{idx}_{screw_idx}_0",
            )
            body.visual(
                Cylinder(radius=0.017, length=0.006),
                origin=Origin(
                    xyz=(-half_d - 0.011, y + 0.16, 0.735 + dz),
                    rpy=(0.0, pi / 2.0, 0.0),
                ),
                material=dark,
                name=f"hinge_screw_{idx}_{screw_idx}_1",
            )

    # Side pivot bosses for the two lid stay arms.
    body.visual(
        Cylinder(radius=0.045, length=0.012),
        origin=Origin(xyz=(-0.115, half_w + 0.006, 0.585), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=metal,
        name="stay_boss_0",
    )
    body.visual(
        Cylinder(radius=0.045, length=0.012),
        origin=Origin(xyz=(-0.115, -half_w - 0.006, 0.585), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=metal,
        name="stay_boss_1",
    )

    # Front compressor/control panel, grille, and indicators.
    body.visual(
        Box((0.012, 0.50, 0.16)),
        origin=Origin(xyz=(half_d + 0.006, -0.42, 0.20)),
        material=dark,
        name="control_panel",
    )
    for i, y in enumerate((-0.565, -0.525, -0.485, -0.445, -0.405, -0.365)):
        body.visual(
            Box((0.014, 0.010, 0.105)),
            origin=Origin(xyz=(half_d + 0.014, y, 0.205)),
            material=metal,
            name=f"grille_slot_{i}",
        )
    body.visual(
        Sphere(radius=0.018),
        origin=Origin(xyz=(half_d + 0.020, -0.225, 0.245)),
        material=green,
        name="power_light",
    )
    body.visual(
        Sphere(radius=0.014),
        origin=Origin(xyz=(half_d + 0.020, -0.185, 0.245)),
        material=amber,
        name="alarm_light",
    )

    # Small leveling feet under the cabinet.
    for ix, x in enumerate((-0.28, 0.28)):
        for iy, y in enumerate((-0.61, 0.61)):
            body.visual(
                Cylinder(radius=0.035, length=0.045),
                origin=Origin(xyz=(x, y, -0.0225)),
                material=dark,
                name=f"leveling_foot_{ix}_{iy}",
            )

    lid = model.part("lid")
    lid.visual(
        Box((0.84, 1.64, 0.080)),
        origin=Origin(xyz=(0.420, 0.0, 0.040)),
        material=gloss_white,
        name="lid_panel",
    )
    # The continuous underside gasket ring is four strips tied into the lid.
    lid.visual(
        Box((0.060, 1.42, 0.018)),
        origin=Origin(xyz=(0.725, 0.0, -0.004)),
        material=gasket,
        name="lid_gasket_front",
    )
    lid.visual(
        Box((0.060, 1.42, 0.018)),
        origin=Origin(xyz=(0.115, 0.0, -0.004)),
        material=gasket,
        name="lid_gasket_rear",
    )
    lid.visual(
        Box((0.610, 0.050, 0.018)),
        origin=Origin(xyz=(0.420, 0.690, -0.004)),
        material=gasket,
        name="lid_gasket_0",
    )
    lid.visual(
        Box((0.610, 0.050, 0.018)),
        origin=Origin(xyz=(0.420, -0.690, -0.004)),
        material=gasket,
        name="lid_gasket_1",
    )
    for idx, y in enumerate((-hinge_y, hinge_y)):
        lid.visual(
            Cylinder(radius=0.025, length=hinge_len),
            origin=Origin(xyz=(0.0, y, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
            material=metal,
            name=f"rear_barrel_{idx}",
        )
        lid.visual(
            Box((0.090, 0.42, 0.010)),
            origin=Origin(xyz=(0.045, y, 0.013)),
            material=metal,
            name=f"lid_hinge_leaf_{idx}",
        )

    # Front pull handle fixed to the moving lid.
    lid.visual(
        Cylinder(radius=0.018, length=0.70),
        origin=Origin(xyz=(0.885, 0.0, 0.030), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=dark,
        name="front_handle",
    )
    for idx, y in enumerate((-0.28, 0.28)):
        lid.visual(
            Box((0.090, 0.040, 0.045)),
            origin=Origin(xyz=(0.840, y, 0.030)),
            material=dark,
            name=f"handle_stanchion_{idx}",
        )

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(-half_d - 0.053, 0.0, 0.830)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=90.0, velocity=1.0, lower=0.0, upper=1.30),
    )

    # Two side stay arms pivot on the cabinet sides.  Their free ends swing up
    # into the open-lid posture to prop the lid.
    for idx, sign in enumerate((1.0, -1.0)):
        arm = model.part(f"stay_arm_{idx}")
        arm.visual(
            Cylinder(radius=0.038, length=0.012),
            origin=Origin(xyz=(0.0, sign * 0.006, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
            material=metal,
            name="pivot_washer",
        )
        arm.visual(
            Box((0.460, 0.012, 0.022)),
            origin=Origin(xyz=(-0.230, sign * 0.018, 0.0)),
            material=metal,
            name="arm_bar",
        )
        arm.visual(
            Cylinder(radius=0.026, length=0.014),
            origin=Origin(xyz=(-0.460, sign * 0.018, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
            material=metal,
            name="lid_roller",
        )
        model.articulation(
            f"body_to_stay_arm_{idx}",
            ArticulationType.REVOLUTE,
            parent=body,
            child=arm,
            origin=Origin(xyz=(-0.115, sign * (half_w + 0.012), 0.585)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=20.0, velocity=1.5, lower=0.0, upper=1.06),
        )

    # A real freezer control is a separate rotating thermostat knob, not a
    # painted-on disk.
    dial = model.part("thermostat_dial")
    dial.visual(
        Cylinder(radius=0.040, length=0.026),
        origin=Origin(xyz=(0.013, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark,
        name="dial_cap",
    )
    dial.visual(
        Box((0.006, 0.010, 0.034)),
        origin=Origin(xyz=(0.028, 0.0, 0.014)),
        material=gloss_white,
        name="dial_pointer",
    )
    model.articulation(
        "body_to_thermostat_dial",
        ArticulationType.REVOLUTE,
        parent=body,
        child=dial,
        origin=Origin(xyz=(half_d + 0.012, -0.285, 0.205)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=2.0, lower=-2.4, upper=2.4),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    stay_0 = object_model.get_part("stay_arm_0")
    stay_1 = object_model.get_part("stay_arm_1")
    dial = object_model.get_part("thermostat_dial")
    lid_hinge = object_model.get_articulation("body_to_lid")
    arm_hinge_0 = object_model.get_articulation("body_to_stay_arm_0")
    arm_hinge_1 = object_model.get_articulation("body_to_stay_arm_1")

    ctx.expect_gap(
        lid,
        body,
        axis="z",
        positive_elem="lid_gasket_front",
        negative_elem="front_top_lip",
        min_gap=0.0,
        max_gap=0.006,
        name="closed lid gasket sits just above front rim",
    )
    ctx.expect_overlap(
        lid,
        body,
        axes="xy",
        elem_a="lid_panel",
        elem_b="front_top_lip",
        min_overlap=0.05,
        name="wide lid covers the cabinet opening",
    )
    ctx.expect_contact(
        stay_0,
        body,
        elem_a="pivot_washer",
        elem_b="stay_boss_0",
        contact_tol=0.001,
        name="stay arm 0 washer is mounted on side boss",
    )
    ctx.expect_contact(
        stay_1,
        body,
        elem_a="pivot_washer",
        elem_b="stay_boss_1",
        contact_tol=0.001,
        name="stay arm 1 washer is mounted on side boss",
    )
    ctx.expect_contact(
        dial,
        body,
        elem_a="dial_cap",
        elem_b="control_panel",
        contact_tol=0.002,
        name="thermostat dial is seated on control panel",
    )

    closed_handle = ctx.part_element_world_aabb(lid, elem="front_handle")
    closed_arm_0 = ctx.part_element_world_aabb(stay_0, elem="lid_roller")
    closed_arm_1 = ctx.part_element_world_aabb(stay_1, elem="lid_roller")
    with ctx.pose({lid_hinge: 1.18, arm_hinge_0: 1.06, arm_hinge_1: 1.06}):
        open_handle = ctx.part_element_world_aabb(lid, elem="front_handle")
        open_arm_0 = ctx.part_element_world_aabb(stay_0, elem="lid_roller")
        open_arm_1 = ctx.part_element_world_aabb(stay_1, elem="lid_roller")
        ctx.expect_contact(
            stay_0,
            lid,
            elem_a="lid_roller",
            elem_b="lid_panel",
            contact_tol=0.0015,
            name="stay arm 0 roller props the open lid",
        )
        ctx.expect_contact(
            stay_1,
            lid,
            elem_a="lid_roller",
            elem_b="lid_panel",
            contact_tol=0.0015,
            name="stay arm 1 roller props the open lid",
        )

    ctx.check(
        "lid opens upward around rear barrels",
        closed_handle is not None
        and open_handle is not None
        and open_handle[0][2] > closed_handle[0][2] + 0.45,
        details=f"closed={closed_handle}, open={open_handle}",
    )
    ctx.check(
        "stay arm 0 swings upward to prop lid",
        closed_arm_0 is not None
        and open_arm_0 is not None
        and open_arm_0[0][2] > closed_arm_0[0][2] + 0.20,
        details=f"closed={closed_arm_0}, open={open_arm_0}",
    )
    ctx.check(
        "stay arm 1 swings upward to prop lid",
        closed_arm_1 is not None
        and open_arm_1 is not None
        and open_arm_1[0][2] > closed_arm_1[0][2] + 0.20,
        details=f"closed={closed_arm_1}, open={open_arm_1}",
    )

    return ctx.report()


object_model = build_object_model()
