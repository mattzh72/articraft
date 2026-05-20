from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pole_mounted_grp_enclosure")

    grp = model.material("warm_grey_grp", rgba=(0.72, 0.74, 0.70, 1.0))
    grp_dark = model.material("shadowed_grp_edges", rgba=(0.48, 0.50, 0.47, 1.0))
    gasket = model.material("black_epdm_gasket", rgba=(0.015, 0.015, 0.013, 1.0))
    galvanized = model.material("galvanized_steel", rgba=(0.62, 0.64, 0.62, 1.0))
    stainless = model.material("brushed_stainless", rgba=(0.78, 0.77, 0.72, 1.0))
    latch_black = model.material("black_latch_grip", rgba=(0.02, 0.02, 0.018, 1.0))

    # Root pole and pole clamp.  The enclosure rear face seats against the flat
    # clamp plates, which are welded/bolted to the vertical galvanized pole.
    pole = model.part("pole")
    pole.visual(
        Cylinder(radius=0.045, length=1.75),
        origin=Origin(xyz=(0.0, 0.0, 0.875)),
        material=galvanized,
        name="round_pole",
    )
    pole.visual(
        Cylinder(radius=0.090, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=galvanized,
        name="base_plate",
    )
    pole.visual(
        Box((0.070, 0.300, 0.040)),
        origin=Origin(xyz=(0.055, 0.0, 1.20)),
        material=galvanized,
        name="lower_clamp_plate",
    )
    pole.visual(
        Box((0.070, 0.300, 0.040)),
        origin=Origin(xyz=(0.055, 0.0, 1.34)),
        material=galvanized,
        name="upper_clamp_plate",
    )
    for idx, z in enumerate((1.20, 1.34)):
        pole.visual(
            Box((0.018, 0.030, 0.025)),
            origin=Origin(xyz=(0.065, -0.130, z)),
            material=galvanized,
            name=f"clamp_nut_{idx}_0",
        )
        pole.visual(
            Box((0.018, 0.030, 0.025)),
            origin=Origin(xyz=(0.065, 0.130, z)),
            material=galvanized,
            name=f"clamp_nut_{idx}_1",
        )

    # Wide hollow molded GRP tray.  The part frame is the center of the top
    # opening; the shell extends downward so an open lid reveals a real cavity.
    body = model.part("body")
    depth = 0.360
    width = 0.840
    height = 0.320
    wall = 0.026
    floor = 0.034
    body.visual(
        Box((depth, width, floor)),
        origin=Origin(xyz=(0.0, 0.0, -height + floor / 2.0)),
        material=grp,
        name="bottom_panel",
    )
    body.visual(
        Box((wall, width, height)),
        origin=Origin(xyz=(-depth / 2.0 + wall / 2.0, 0.0, -height / 2.0)),
        material=grp,
        name="rear_wall",
    )
    body.visual(
        Box((wall, width, height)),
        origin=Origin(xyz=(depth / 2.0 - wall / 2.0, 0.0, -height / 2.0)),
        material=grp,
        name="front_wall",
    )
    body.visual(
        Box((depth - 2.0 * wall, wall, height)),
        origin=Origin(xyz=(0.0, -width / 2.0 + wall / 2.0, -height / 2.0)),
        material=grp,
        name="side_wall_0",
    )
    body.visual(
        Box((depth - 2.0 * wall, wall, height)),
        origin=Origin(xyz=(0.0, width / 2.0 - wall / 2.0, -height / 2.0)),
        material=grp,
        name="side_wall_1",
    )
    for sx in (-1.0, 1.0):
        for sy in (-1.0, 1.0):
            body.visual(
                Cylinder(radius=0.020, length=height),
                origin=Origin(xyz=(sx * (depth / 2.0 - wall), sy * (width / 2.0 - wall), -height / 2.0)),
                material=grp,
                name=f"rounded_corner_{0 if sx < 0 else 1}_{0 if sy < 0 else 1}",
            )
    body.visual(
        Box((0.012, 0.670, 0.030)),
        origin=Origin(xyz=(depth / 2.0 + 0.0055, 0.0, -0.175)),
        material=grp_dark,
        name="front_reinforcing_rib",
    )
    body.visual(
        Box((0.012, 0.600, 0.026)),
        origin=Origin(xyz=(depth / 2.0 + 0.0055, 0.0, -0.075)),
        material=grp_dark,
        name="front_upper_rib",
    )
    body.visual(
        Box((0.030, 0.170, 0.070)),
        origin=Origin(xyz=(depth / 2.0 - 0.005, 0.0, -0.060)),
        material=stainless,
        name="latch_strike",
    )

    # Fixed hinge yokes at the rear edge.  Each pair of plates straddles one
    # moving lid knuckle without interpenetrating it.
    hinge_y = 0.285
    knuckle_len = 0.075
    lug_gap = 0.006
    for side, cy in enumerate((-hinge_y, hinge_y)):
        for lug, sign in enumerate((-1.0, 1.0)):
            body.visual(
                Box((0.052, 0.012, 0.056)),
                origin=Origin(
                    xyz=(
                        -depth / 2.0 - 0.018,
                        cy + sign * (knuckle_len / 2.0 + lug_gap),
                        0.026,
                    )
                ),
                material=stainless,
                name=f"hinge_yoke_{side}_{lug}",
            )

    model.articulation(
        "pole_to_body",
        ArticulationType.FIXED,
        parent=pole,
        child=body,
        origin=Origin(xyz=(0.270, 0.0, 1.40)),
    )

    # Hinged lid.  Its child frame is the rear hinge axis, so the broad cover
    # extends in local +X across the opening and positive joint motion opens up.
    lid = model.part("lid")
    lid_depth = depth + 0.046
    lid_width = width + 0.050
    hinge_axis_z = 0.026
    lid.visual(
        Box((0.330, 0.440, 0.032)),
        origin=Origin(xyz=(0.147, 0.0, -hinge_axis_z + 0.016)),
        material=grp,
        name="lid_panel",
    )
    lid.visual(
        Box((lid_depth, 0.095, 0.032)),
        origin=Origin(xyz=(lid_depth / 2.0 - 0.018, -0.398, -hinge_axis_z + 0.016)),
        material=grp,
        name="lid_side_panel_0",
    )
    lid.visual(
        Box((lid_depth, 0.095, 0.032)),
        origin=Origin(xyz=(lid_depth / 2.0 - 0.018, 0.398, -hinge_axis_z + 0.016)),
        material=grp,
        name="lid_side_panel_1",
    )
    lid.visual(
        Box((0.030, 0.440, 0.055)),
        origin=Origin(xyz=(-(0.018 + 0.003), 0.0, -hinge_axis_z - 0.010)),
        material=grp,
        name="rear_skirt",
    )
    lid.visual(
        Box((0.030, 0.095, 0.055)),
        origin=Origin(xyz=(-(0.018 + 0.003), -0.398, -hinge_axis_z - 0.010)),
        material=grp,
        name="rear_skirt_side_0",
    )
    lid.visual(
        Box((0.030, 0.095, 0.055)),
        origin=Origin(xyz=(-(0.018 + 0.003), 0.398, -hinge_axis_z - 0.010)),
        material=grp,
        name="rear_skirt_side_1",
    )
    lid.visual(
        Box((0.085, 0.350, 0.032)),
        origin=Origin(xyz=(0.357, -0.270, -hinge_axis_z + 0.016)),
        material=grp,
        name="lid_front_panel_0",
    )
    lid.visual(
        Box((0.085, 0.350, 0.032)),
        origin=Origin(xyz=(0.357, 0.270, -hinge_axis_z + 0.016)),
        material=grp,
        name="lid_front_panel_1",
    )
    lid.visual(
        Box((0.030, 0.350, 0.055)),
        origin=Origin(xyz=(depth + 0.032, -0.270, -hinge_axis_z - 0.010)),
        material=grp,
        name="front_skirt",
    )
    lid.visual(
        Box((0.030, 0.350, 0.055)),
        origin=Origin(xyz=(depth + 0.032, 0.270, -hinge_axis_z - 0.010)),
        material=grp,
        name="front_skirt_side",
    )
    lid.visual(
        Box((lid_depth, 0.020, 0.045)),
        origin=Origin(xyz=(lid_depth / 2.0 - 0.018, -lid_width / 2.0 + 0.010, -hinge_axis_z - 0.008)),
        material=grp,
        name="side_skirt_0",
    )
    lid.visual(
        Box((lid_depth, 0.020, 0.045)),
        origin=Origin(xyz=(lid_depth / 2.0 - 0.018, lid_width / 2.0 - 0.010, -hinge_axis_z - 0.008)),
        material=grp,
        name="side_skirt_1",
    )
    lid.visual(
        Box((0.115, 0.055, 0.010)),
        origin=Origin(xyz=(0.105, 0.0, 0.0105)),
        material=grp_dark,
        name="raised_center_rib",
    )
    lid.visual(
        Box((0.044, 0.700, 0.010)),
        origin=Origin(xyz=(0.185, 0.0, 0.0105)),
        material=grp_dark,
        name="cross_rib",
    )
    # Continuous gasket strips are tucked just inside the body walls and overlap
    # the lid underside slightly, representing a compressed EPDM seal.
    lid.visual(
        Box((0.290, 0.018, 0.014)),
        origin=Origin(xyz=(0.180, -0.365, -hinge_axis_z - 0.006)),
        material=gasket,
        name="gasket_side_0",
    )
    lid.visual(
        Box((0.290, 0.018, 0.014)),
        origin=Origin(xyz=(0.180, 0.365, -hinge_axis_z - 0.006)),
        material=gasket,
        name="gasket_side_1",
    )
    lid.visual(
        Box((0.018, 0.730, 0.014)),
        origin=Origin(xyz=(0.040, 0.0, -hinge_axis_z - 0.006)),
        material=gasket,
        name="gasket_rear",
    )
    lid.visual(
        Box((0.018, 0.730, 0.014)),
        origin=Origin(xyz=(0.320, 0.0, -hinge_axis_z - 0.006)),
        material=gasket,
        name="gasket_front",
    )
    for side, cy in enumerate((-hinge_y, hinge_y)):
        lid.visual(
            Box((0.090, 0.150, 0.012)),
            origin=Origin(xyz=(0.075, cy, -0.006)),
            material=stainless,
            name=f"hinge_bridge_{side}",
        )
        lid.visual(
            Cylinder(radius=0.016, length=knuckle_len),
            origin=Origin(xyz=(0.0, cy, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=stainless,
            name=f"hinge_knuckle_{side}",
        )
        lid.visual(
            Box((0.070, knuckle_len, 0.012)),
            origin=Origin(xyz=(0.026, cy, -0.006)),
            material=stainless,
            name=f"hinge_leaf_{side}",
        )

    lid_joint = model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(-depth / 2.0, 0.0, hinge_axis_z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.8, lower=0.0, upper=1.22),
    )

    # Over-centre compression latch on the lid front.  The handle and cam foot
    # are one moving part rotating on a horizontal pin across the lid width.
    latch = model.part("latch")
    latch.visual(
        Cylinder(radius=0.011, length=0.092),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=stainless,
        name="pivot_pin",
    )
    latch.visual(
        Box((0.125, 0.042, 0.014)),
        origin=Origin(xyz=(-0.070, 0.0, 0.0150)),
        material=latch_black,
        name="cam_handle",
    )
    latch.visual(
        Box((0.030, 0.034, 0.078)),
        origin=Origin(xyz=(0.020, 0.0, -0.042)),
        material=stainless,
        name="cam_foot",
    )
    latch.visual(
        Cylinder(radius=0.018, length=0.014),
        origin=Origin(xyz=(-0.130, 0.0, 0.0150), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=latch_black,
        name="thumb_pad",
    )
    # Non-moving bearing ears on the lid, placed outside the latch pin ends.
    lid.visual(
        Box((0.022, 0.012, 0.040)),
        origin=Origin(xyz=(0.365, -0.055, -0.004)),
        material=stainless,
        name="latch_ear_0",
    )
    lid.visual(
        Box((0.022, 0.045, 0.040)),
        origin=Origin(xyz=(0.365, -0.078, -0.004)),
        material=stainless,
        name="latch_ear_bridge_0",
    )
    lid.visual(
        Box((0.022, 0.012, 0.040)),
        origin=Origin(xyz=(0.365, 0.055, -0.004)),
        material=stainless,
        name="latch_ear_1",
    )
    lid.visual(
        Box((0.022, 0.045, 0.040)),
        origin=Origin(xyz=(0.365, 0.078, -0.004)),
        material=stainless,
        name="latch_ear_bridge_1",
    )

    model.articulation(
        "lid_to_latch",
        ArticulationType.REVOLUTE,
        parent=lid,
        child=latch,
        origin=Origin(xyz=(0.365, 0.0, 0.009)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.5, velocity=3.0, lower=0.0, upper=1.35),
    )

    # Store design dimensions for tests and probes without depending on globals.
    model.meta["lid_open_upper"] = lid_joint.motion_limits.upper
    model.meta["latch_open_upper"] = 1.35
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    pole = object_model.get_part("pole")
    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    latch = object_model.get_part("latch")
    lid_joint = object_model.get_articulation("body_to_lid")
    latch_joint = object_model.get_articulation("lid_to_latch")

    ctx.expect_gap(
        body,
        pole,
        axis="x",
        positive_elem="rear_wall",
        negative_elem="upper_clamp_plate",
        max_gap=0.001,
        max_penetration=0.0,
        name="enclosure rear seats on pole clamp",
    )
    ctx.expect_gap(
        lid,
        body,
        axis="z",
        positive_elem="lid_panel",
        negative_elem="front_wall",
        max_gap=0.001,
        max_penetration=0.0,
        name="lid closes onto body rim",
    )
    ctx.expect_overlap(
        lid,
        body,
        axes="xy",
        elem_a="lid_panel",
        elem_b="bottom_panel",
        min_overlap=0.30,
        name="wide lid covers rectangular enclosure body",
    )
    ctx.expect_gap(
        latch,
        body,
        axis="x",
        positive_elem="cam_foot",
        negative_elem="latch_strike",
        max_gap=0.018,
        max_penetration=0.0,
        name="cam foot sits just in front of strike",
    )

    closed_lid = ctx.part_element_world_aabb(lid, elem="front_skirt")
    with ctx.pose({lid_joint: object_model.meta["lid_open_upper"]}):
        open_lid = ctx.part_element_world_aabb(lid, elem="front_skirt")
    ctx.check(
        "rear hinge lifts lid front edge",
        closed_lid is not None
        and open_lid is not None
        and open_lid[1][2] > closed_lid[1][2] + 0.22,
        details=f"closed={closed_lid}, open={open_lid}",
    )

    closed_latch = ctx.part_element_world_aabb(latch, elem="cam_handle")
    with ctx.pose({latch_joint: object_model.meta["latch_open_upper"]}):
        open_latch = ctx.part_element_world_aabb(latch, elem="cam_handle")
    ctx.check(
        "cam-over latch handle rotates upward",
        closed_latch is not None
        and open_latch is not None
        and open_latch[1][2] > closed_latch[1][2] + 0.06,
        details=f"closed={closed_latch}, open={open_latch}",
    )

    return ctx.report()


object_model = build_object_model()
