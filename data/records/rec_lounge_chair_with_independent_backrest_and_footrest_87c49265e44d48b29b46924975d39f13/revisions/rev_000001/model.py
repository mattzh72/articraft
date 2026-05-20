from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _rounded_panel_mesh(name: str, *, width: float, height: float, thickness: float, axis: str = "z"):
    radius = min(width, height) * 0.10
    geom = ExtrudeGeometry(
        rounded_rect_profile(width, height, radius, corner_segments=8),
        thickness,
        center=True,
    )
    if axis == "x":
        geom.rotate_y(math.pi / 2.0)
    elif axis == "y":
        geom.rotate_x(-math.pi / 2.0)
    return mesh_from_geometry(geom, name)


def _aabb_center(aabb):
    return tuple((lower + upper) * 0.5 for lower, upper in zip(aabb[0], aabb[1]))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="slim_lounge_chair")

    wood = model.material("walnut_wood", rgba=(0.55, 0.34, 0.20, 1.0))
    upholstery = model.material("warm_tan_upholstery", rgba=(0.74, 0.67, 0.56, 1.0))
    dark_upholstery = model.material("charcoal_fabric", rgba=(0.28, 0.28, 0.30, 1.0))
    metal = model.material("dark_metal", rgba=(0.22, 0.23, 0.25, 1.0))

    seat_cushion_mesh = _rounded_panel_mesh(
        "lounge_seat_cushion",
        width=0.56,
        height=0.57,
        thickness=0.072,
    )
    back_cushion_mesh = _rounded_panel_mesh(
        "lounge_back_cushion",
        width=0.52,
        height=0.62,
        thickness=0.045,
        axis="x",
    )
    calf_board_mesh = _rounded_panel_mesh(
        "lounge_calf_rest_board",
        width=0.48,
        height=0.30,
        thickness=0.018,
    )

    chair_body = model.part("chair_body")
    chair_body.inertial = Inertial.from_geometry(
        Box((0.90, 0.68, 0.90)),
        mass=28.0,
        origin=Origin(xyz=(-0.02, 0.0, 0.45)),
    )

    for side_name, side_y in (("left", 0.315), ("right", -0.315)):
        rear_side_y = 0.340 if side_y > 0.0 else -0.340
        chair_body.visual(
            Box((0.045, 0.030, 0.390)),
            origin=Origin(xyz=(0.215, side_y, 0.195)),
            material=wood,
            name=f"{side_name}_front_leg",
        )
        chair_body.visual(
            Box((0.055, 0.030, 0.840)),
            origin=Origin(xyz=(-0.255, rear_side_y, 0.410), rpy=(0.0, -0.30, 0.0)),
            material=wood,
            name=f"{side_name}_rear_post",
        )
        chair_body.visual(
            Box((0.600, 0.034, 0.034)),
            origin=Origin(xyz=(-0.015, rear_side_y, 0.545), rpy=(0.0, -0.06, 0.0)),
            material=wood,
            name=f"{side_name}_arm_rail",
        )
        chair_body.visual(
            Box((0.580, 0.030, 0.038)),
            origin=Origin(xyz=(0.000, side_y, 0.050)),
            material=wood,
            name=f"{side_name}_lower_rail",
        )
        chair_body.visual(
            Box((0.480, 0.030, 0.034)),
            origin=Origin(xyz=(0.020, side_y, 0.302)),
            material=wood,
            name=f"{side_name}_seat_rail",
        )

    chair_body.visual(
        Box((0.040, 0.600, 0.030)),
        origin=Origin(xyz=(0.270, 0.0, 0.318)),
        material=wood,
        name="front_rail",
    )
    chair_body.visual(
        Box((0.040, 0.600, 0.030)),
        origin=Origin(xyz=(-0.255, 0.0, 0.305)),
        material=wood,
        name="rear_rail",
    )
    chair_body.visual(
        Box((0.540, 0.560, 0.020)),
        origin=Origin(xyz=(0.005, 0.0, 0.305)),
        material=wood,
        name="seat_deck",
    )
    chair_body.visual(
        seat_cushion_mesh,
        origin=Origin(xyz=(0.000, 0.0, 0.351)),
        material=upholstery,
        name="seat_cushion",
    )
    chair_body.visual(
        Box((0.034, 0.008, 0.064)),
        origin=Origin(xyz=(-0.255, 0.322, 0.348)),
        material=metal,
        name="left_pivot_bracket",
    )
    chair_body.visual(
        Box((0.034, 0.008, 0.064)),
        origin=Origin(xyz=(-0.255, -0.322, 0.348)),
        material=metal,
        name="right_pivot_bracket",
    )
    for side_name, side_y in (("left", 0.240), ("right", -0.240)):
        mount_y = side_y + 0.018 if side_y > 0.0 else side_y - 0.018
        chair_body.visual(
            Box((0.380, 0.018, 0.016)),
            origin=Origin(xyz=(0.130, side_y, 0.249)),
            material=metal,
            name=f"{side_name}_guide_lower",
        )
        chair_body.visual(
            Box((0.380, 0.018, 0.008)),
            origin=Origin(xyz=(0.130, side_y, 0.281)),
            material=metal,
            name=f"{side_name}_guide_upper",
        )
        chair_body.visual(
            Box((0.020, 0.018, 0.054)),
            origin=Origin(xyz=(-0.050, mount_y, 0.268)),
            material=metal,
            name=f"{side_name}_guide_mount",
        )

    back_panel = model.part("back_panel")
    back_panel.inertial = Inertial.from_geometry(
        Box((0.18, 0.58, 0.78)),
        mass=8.5,
        origin=Origin(xyz=(-0.05, 0.0, 0.44)),
    )
    back_panel.visual(
        Cylinder(radius=0.017, length=0.022),
        origin=Origin(xyz=(0.000, 0.307, 0.000), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=metal,
        name="left_pivot_barrel",
    )
    back_panel.visual(
        Cylinder(radius=0.017, length=0.022),
        origin=Origin(xyz=(0.000, -0.307, 0.000), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=metal,
        name="right_pivot_barrel",
    )
    back_panel.visual(
        Box((0.070, 0.014, 0.110)),
        origin=Origin(xyz=(-0.035, 0.307, 0.055)),
        material=metal,
        name="left_hinge_cheek",
    )
    back_panel.visual(
        Box((0.070, 0.014, 0.110)),
        origin=Origin(xyz=(-0.035, -0.307, 0.055)),
        material=metal,
        name="right_hinge_cheek",
    )
    back_panel.visual(
        Box((0.032, 0.600, 0.060)),
        origin=Origin(xyz=(-0.060, 0.0, 0.132), rpy=(0.0, -0.10, 0.0)),
        material=wood,
        name="back_lower_spar",
    )
    back_panel.visual(
        Box((0.036, 0.016, 0.610)),
        origin=Origin(xyz=(-0.060, 0.300, 0.430), rpy=(0.0, -0.10, 0.0)),
        material=wood,
        name="left_back_stile",
    )
    back_panel.visual(
        Box((0.036, 0.016, 0.610)),
        origin=Origin(xyz=(-0.060, -0.300, 0.430), rpy=(0.0, -0.10, 0.0)),
        material=wood,
        name="right_back_stile",
    )
    back_panel.visual(
        Box((0.036, 0.600, 0.045)),
        origin=Origin(xyz=(-0.075, 0.0, 0.728), rpy=(0.0, -0.10, 0.0)),
        material=wood,
        name="back_top_rail",
    )
    back_panel.visual(
        back_cushion_mesh,
        origin=Origin(xyz=(-0.060, 0.0, 0.430), rpy=(0.0, -0.10, 0.0)),
        material=dark_upholstery,
        name="back_cushion",
    )

    calf_rest = model.part("calf_rest")
    calf_rest.inertial = Inertial.from_geometry(
        Box((0.52, 0.50, 0.07)),
        mass=3.2,
        origin=Origin(xyz=(0.260, 0.0, 0.028)),
    )
    calf_rest.visual(
        Box((0.440, 0.012, 0.016)),
        origin=Origin(xyz=(0.220, 0.240, 0.000)),
        material=metal,
        name="left_slider",
    )
    calf_rest.visual(
        Box((0.440, 0.012, 0.016)),
        origin=Origin(xyz=(0.220, -0.240, 0.000)),
        material=metal,
        name="right_slider",
    )
    calf_rest.visual(
        Box((0.180, 0.500, 0.014)),
        origin=Origin(xyz=(0.200, 0.0, -0.001)),
        material=metal,
        name="slider_crossbar",
    )
    calf_rest.visual(
        calf_board_mesh,
        origin=Origin(xyz=(0.310, 0.0, 0.015)),
        material=upholstery,
        name="calf_board",
    )

    model.articulation(
        "seat_to_back",
        ArticulationType.REVOLUTE,
        parent=chair_body,
        child=back_panel,
        origin=Origin(xyz=(-0.255, 0.0, 0.348)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.2,
            lower=0.0,
            upper=0.55,
        ),
    )
    model.articulation(
        "seat_to_calf_rest",
        ArticulationType.PRISMATIC,
        parent=chair_body,
        child=calf_rest,
        origin=Origin(xyz=(-0.020, 0.0, 0.265)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=0.20,
            lower=0.0,
            upper=0.18,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    chair_body = object_model.get_part("chair_body")
    back_panel = object_model.get_part("back_panel")
    calf_rest = object_model.get_part("calf_rest")
    seat_to_back = object_model.get_articulation("seat_to_back")
    seat_to_calf_rest = object_model.get_articulation("seat_to_calf_rest")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "back pivot axis is horizontal",
        tuple(round(value, 6) for value in seat_to_back.axis) == (0.0, -1.0, 0.0),
        f"Expected lateral rear pivot axis, got {seat_to_back.axis!r}",
    )
    ctx.check(
        "calf rest slides forward",
        tuple(round(value, 6) for value in seat_to_calf_rest.axis) == (1.0, 0.0, 0.0),
        f"Expected forward slide axis, got {seat_to_calf_rest.axis!r}",
    )

    ctx.expect_contact(
        back_panel,
        chair_body,
        elem_a="left_pivot_barrel",
        elem_b="left_pivot_bracket",
    )
    ctx.expect_contact(
        back_panel,
        chair_body,
        elem_a="right_pivot_barrel",
        elem_b="right_pivot_bracket",
    )
    ctx.expect_contact(
        calf_rest,
        chair_body,
        elem_a="left_slider",
        elem_b="left_guide_lower",
    )
    ctx.expect_contact(
        calf_rest,
        chair_body,
        elem_a="right_slider",
        elem_b="right_guide_lower",
    )
    ctx.expect_gap(
        chair_body,
        calf_rest,
        axis="z",
        positive_elem="front_rail",
        negative_elem="calf_board",
        min_gap=0.0005,
        max_gap=0.020,
        name="calf board clears front rail underside",
    )
    ctx.expect_overlap(
        calf_rest,
        chair_body,
        axes="xy",
        elem_a="left_slider",
        elem_b="left_guide_lower",
        min_overlap=0.010,
        name="left slider stays registered in guide at rest",
    )
    ctx.expect_overlap(
        calf_rest,
        chair_body,
        axes="xy",
        elem_a="right_slider",
        elem_b="right_guide_lower",
        min_overlap=0.010,
        name="right slider stays registered in guide at rest",
    )

    rest_back_top = ctx.part_element_world_aabb(back_panel, elem="back_top_rail")
    rest_calf_board = ctx.part_element_world_aabb(calf_rest, elem="calf_board")
    if rest_back_top is None:
        ctx.fail("back top rail exists", "Missing back_top_rail world AABB")
        return ctx.report()
    if rest_calf_board is None:
        ctx.fail("calf board exists", "Missing calf_board world AABB")
        return ctx.report()

    rest_back_center = _aabb_center(rest_back_top)
    rest_calf_center = _aabb_center(rest_calf_board)

    with ctx.pose({seat_to_back: 0.50}):
        reclined_back_top = ctx.part_element_world_aabb(back_panel, elem="back_top_rail")
        reclined_calf_board = ctx.part_element_world_aabb(calf_rest, elem="calf_board")
        if reclined_back_top is None or reclined_calf_board is None:
            ctx.fail("posed back and calf rest resolve", "Expected posed AABBs for articulated parts")
        else:
            reclined_back_center = _aabb_center(reclined_back_top)
            reclined_calf_center = _aabb_center(reclined_calf_board)
            ctx.check(
                "back reclines rearward",
                reclined_back_center[0] < rest_back_center[0] - 0.14,
                (
                    "Expected back top rail to move rearward under recline; "
                    f"rest x={rest_back_center[0]:.4f}, reclined x={reclined_back_center[0]:.4f}"
                ),
            )
            ctx.check(
                "back recline does not drag calf rest",
                abs(reclined_calf_center[0] - rest_calf_center[0]) < 1e-6,
                (
                    "Calf rest should remain independent of back motion; "
                    f"rest x={rest_calf_center[0]:.6f}, reclined x={reclined_calf_center[0]:.6f}"
                ),
            )

    with ctx.pose({seat_to_calf_rest: 0.18}):
        extended_calf_board = ctx.part_element_world_aabb(calf_rest, elem="calf_board")
        extended_back_top = ctx.part_element_world_aabb(back_panel, elem="back_top_rail")
        if extended_calf_board is None or extended_back_top is None:
            ctx.fail("extended calf rest and back resolve", "Expected extended-pose AABBs for articulated parts")
        else:
            extended_calf_center = _aabb_center(extended_calf_board)
            extended_back_center = _aabb_center(extended_back_top)
            ctx.check(
                "calf rest extends forward",
                extended_calf_center[0] > rest_calf_center[0] + 0.17,
                (
                    "Expected calf board to slide forward; "
                    f"rest x={rest_calf_center[0]:.4f}, extended x={extended_calf_center[0]:.4f}"
                ),
            )
            ctx.check(
                "calf rest keeps constant elevation while sliding",
                abs(extended_calf_center[2] - rest_calf_center[2]) < 0.002,
                (
                    "Calf board should translate without lifting; "
                    f"rest z={rest_calf_center[2]:.4f}, extended z={extended_calf_center[2]:.4f}"
                ),
            )
            ctx.check(
                "calf rest slide does not move back",
                abs(extended_back_center[0] - rest_back_center[0]) < 1e-6,
                (
                    "Back should remain independent of calf-rest travel; "
                    f"rest x={rest_back_center[0]:.6f}, slid x={extended_back_center[0]:.6f}"
                ),
            )
        ctx.expect_overlap(
            calf_rest,
            chair_body,
            axes="x",
            elem_a="left_slider",
            elem_b="left_guide_lower",
            min_overlap=0.14,
            name="left slider remains captured when extended",
        )
        ctx.expect_overlap(
            calf_rest,
            chair_body,
            axes="x",
            elem_a="right_slider",
            elem_b="right_guide_lower",
            min_overlap=0.14,
            name="right slider remains captured when extended",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
