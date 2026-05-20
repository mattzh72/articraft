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
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wide_flatbed_scanner_printer")

    warm_white = Material("warm_white", rgba=(0.86, 0.86, 0.82, 1.0))
    light_gray = Material("light_gray", rgba=(0.58, 0.60, 0.60, 1.0))
    dark_gray = Material("dark_gray", rgba=(0.12, 0.13, 0.14, 1.0))
    black = Material("black_rubber", rgba=(0.015, 0.016, 0.018, 1.0))
    glass = Material("smoked_scanner_glass", rgba=(0.08, 0.17, 0.22, 0.55))
    paper = Material("paper_white", rgba=(0.97, 0.97, 0.94, 1.0))
    rail_gray = Material("rail_gray", rgba=(0.42, 0.43, 0.42, 1.0))

    body = model.part("body")
    # A wide printer body built as a real open chassis rather than a solid block,
    # leaving a clear paper path and tray slot at the front.
    body.visual(
        Box((0.72, 0.46, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 0.0175)),
        material=warm_white,
        name="bottom_shell",
    )
    body.visual(
        Box((0.035, 0.46, 0.125)),
        origin=Origin(xyz=(-0.3425, 0.0, 0.095)),
        material=warm_white,
        name="side_shell_0",
    )
    body.visual(
        Box((0.035, 0.46, 0.125)),
        origin=Origin(xyz=(0.3425, 0.0, 0.095)),
        material=warm_white,
        name="side_shell_1",
    )
    body.visual(
        Box((0.72, 0.035, 0.125)),
        origin=Origin(xyz=(0.0, 0.2125, 0.095)),
        material=warm_white,
        name="rear_shell",
    )
    body.visual(
        Box((0.72, 0.46, 0.030)),
        origin=Origin(xyz=(0.0, 0.0, 0.145)),
        material=warm_white,
        name="scan_deck",
    )
    body.visual(
        Box((0.72, 0.035, 0.035)),
        origin=Origin(xyz=(0.0, -0.2125, 0.0175)),
        material=warm_white,
        name="front_lower_lip",
    )
    body.visual(
        Box((0.72, 0.035, 0.050)),
        origin=Origin(xyz=(0.0, -0.2125, 0.1050)),
        material=warm_white,
        name="front_upper_fascia",
    )
    body.visual(
        Box((0.56, 0.007, 0.004)),
        origin=Origin(xyz=(0.0, -0.231, 0.078)),
        material=black,
        name="front_paper_slot",
    )

    # Flatbed scanner glass and raised surround visible when the lid is lifted.
    body.visual(
        Box((0.50, 0.285, 0.004)),
        origin=Origin(xyz=(-0.035, 0.005, 0.162)),
        material=glass,
        name="scan_glass",
    )
    body.visual(
        Box((0.57, 0.025, 0.010)),
        origin=Origin(xyz=(-0.035, -0.1525, 0.161)),
        material=light_gray,
        name="scan_bezel_front",
    )
    body.visual(
        Box((0.57, 0.025, 0.010)),
        origin=Origin(xyz=(-0.035, 0.1625, 0.161)),
        material=light_gray,
        name="scan_bezel_rear",
    )
    body.visual(
        Box((0.025, 0.315, 0.010)),
        origin=Origin(xyz=(-0.3325, 0.005, 0.161)),
        material=light_gray,
        name="scan_bezel_side_0",
    )
    body.visual(
        Box((0.025, 0.315, 0.010)),
        origin=Origin(xyz=(0.2625, 0.005, 0.161)),
        material=light_gray,
        name="scan_bezel_side_1",
    )
    body.visual(
        Box((0.20, 0.070, 0.008)),
        origin=Origin(xyz=(0.235, -0.160, 0.160)),
        material=dark_gray,
        name="touch_panel",
    )
    body.visual(
        Box((0.015, 0.015, 0.004)),
        origin=Origin(xyz=(0.165, -0.162, 0.162)),
        material=black,
        name="status_dot",
    )

    # Two exposed rear barrel hinge mounts, with stationary outer knuckles on
    # the body. The moving center knuckles are authored on the lid.
    hinge_centers = (-0.235, 0.235)
    for index, x0 in enumerate(hinge_centers):
        body.visual(
            Box((0.095, 0.030, 0.006)),
            origin=Origin(xyz=(x0, 0.214, 0.163)),
            material=rail_gray,
            name=f"hinge_leaf_{index}",
        )
        body.visual(
            Box((0.024, 0.006, 0.020)),
            origin=Origin(xyz=(x0 - 0.030, 0.219, 0.170)),
            material=rail_gray,
            name=f"hinge_web_{index}_0",
        )
        body.visual(
            Box((0.024, 0.006, 0.020)),
            origin=Origin(xyz=(x0 + 0.030, 0.219, 0.170)),
            material=rail_gray,
            name=f"hinge_web_{index}_1",
        )
        body.visual(
            Cylinder(radius=0.010, length=0.024),
            origin=Origin(xyz=(x0 - 0.030, 0.230, 0.181), rpy=(0.0, pi / 2.0, 0.0)),
            material=rail_gray,
            name=f"hinge_outer_knuckle_{index}_0",
        )
        body.visual(
            Cylinder(radius=0.010, length=0.024),
            origin=Origin(xyz=(x0 + 0.030, 0.230, 0.181), rpy=(0.0, pi / 2.0, 0.0)),
            material=rail_gray,
            name=f"hinge_outer_knuckle_{index}_1",
        )

    lid = model.part("lid")
    # The lid part frame is on the shared rear hinge axis. In the closed pose the
    # slab extends forward along local -Y from that axis.
    lid.visual(
        Box((0.685, 0.430, 0.028)),
        origin=Origin(xyz=(0.0, -0.230, 0.0)),
        material=warm_white,
        name="lid_shell",
    )
    lid.visual(
        Box((0.64, 0.030, 0.018)),
        origin=Origin(xyz=(0.0, -0.388, -0.002)),
        material=light_gray,
        name="front_grip",
    )
    lid.visual(
        Box((0.46, 0.250, 0.004)),
        origin=Origin(xyz=(-0.035, -0.205, -0.015)),
        material=dark_gray,
        name="lid_underside_pad",
    )
    for index, x0 in enumerate(hinge_centers):
        lid.visual(
            Box((0.028, 0.048, 0.006)),
            origin=Origin(xyz=(x0, -0.024, -0.012)),
            material=rail_gray,
            name=f"lid_hinge_leaf_{index}",
        )
        lid.visual(
            Cylinder(radius=0.009, length=0.029),
            origin=Origin(xyz=(x0, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
            material=rail_gray,
            name=f"hinge_center_knuckle_{index}",
        )

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, 0.230, 0.181)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.2, lower=0.0, upper=1.20),
    )

    tray = model.part("paper_tray")
    # The tray frame starts at the front slot; hidden length extends back into
    # the body so the tray remains captured at full extension.
    tray.visual(
        Box((0.585, 0.370, 0.014)),
        origin=Origin(xyz=(0.0, 0.065, 0.000)),
        material=light_gray,
        name="tray_panel",
    )
    tray.visual(
        Box((0.585, 0.030, 0.038)),
        origin=Origin(xyz=(0.0, -0.135, 0.019)),
        material=warm_white,
        name="tray_front_lip",
    )
    tray.visual(
        Box((0.030, 0.310, 0.018)),
        origin=Origin(xyz=(-0.245, 0.080, 0.016)),
        material=rail_gray,
        name="tray_side_rail_0",
    )
    tray.visual(
        Box((0.030, 0.310, 0.018)),
        origin=Origin(xyz=(0.245, 0.080, 0.016)),
        material=rail_gray,
        name="tray_side_rail_1",
    )
    tray.visual(
        Box((0.500, 0.235, 0.006)),
        origin=Origin(xyz=(0.0, -0.010, 0.013)),
        material=paper,
        name="paper_stack",
    )
    tray.visual(
        Box((0.440, 0.020, 0.016)),
        origin=Origin(xyz=(0.0, 0.188, 0.014)),
        material=rail_gray,
        name="paper_stop",
    )

    model.articulation(
        "body_to_paper_tray",
        ArticulationType.PRISMATIC,
        parent=body,
        child=tray,
        origin=Origin(xyz=(0.0, -0.230, 0.042)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=0.35, lower=0.0, upper=0.220),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    tray = object_model.get_part("paper_tray")
    lid_hinge = object_model.get_articulation("body_to_lid")
    tray_slide = object_model.get_articulation("body_to_paper_tray")

    ctx.expect_gap(
        lid,
        body,
        axis="z",
        positive_elem="lid_shell",
        negative_elem="scan_bezel_front",
        min_gap=0.0,
        max_gap=0.004,
        name="closed lid rests just above scanner bezel",
    )
    ctx.expect_within(
        tray,
        body,
        axes="x",
        inner_elem="tray_panel",
        outer_elem="front_paper_slot",
        margin=0.055,
        name="paper tray is centered in front feed slot",
    )
    ctx.expect_overlap(
        tray,
        body,
        axes="y",
        elem_a="tray_panel",
        elem_b="front_lower_lip",
        min_overlap=0.030,
        name="collapsed tray remains inserted through the front slot",
    )

    rest_lid_aabb = ctx.part_element_world_aabb(lid, elem="lid_shell")
    rest_tray_pos = ctx.part_world_position(tray)
    with ctx.pose({lid_hinge: 1.05, tray_slide: 0.220}):
        opened_lid_aabb = ctx.part_element_world_aabb(lid, elem="lid_shell")
        extended_tray_pos = ctx.part_world_position(tray)
        ctx.expect_overlap(
            tray,
            body,
            axes="y",
            elem_a="tray_panel",
            elem_b="front_lower_lip",
            min_overlap=0.015,
            name="extended tray still has retained insertion",
        )

    ctx.check(
        "scan lid opens upward from rear barrel hinges",
        rest_lid_aabb is not None
        and opened_lid_aabb is not None
        and opened_lid_aabb[1][2] > rest_lid_aabb[1][2] + 0.20,
        details=f"rest_lid_aabb={rest_lid_aabb}, opened_lid_aabb={opened_lid_aabb}",
    )
    ctx.check(
        "paper tray slides out toward the front",
        rest_tray_pos is not None
        and extended_tray_pos is not None
        and extended_tray_pos[1] < rest_tray_pos[1] - 0.18,
        details=f"rest_tray_pos={rest_tray_pos}, extended_tray_pos={extended_tray_pos}",
    )

    return ctx.report()


object_model = build_object_model()
