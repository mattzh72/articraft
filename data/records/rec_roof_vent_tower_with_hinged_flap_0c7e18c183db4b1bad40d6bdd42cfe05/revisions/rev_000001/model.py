from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

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
    model = ArticulatedObject(name="roof_vent_tower_hinged_flap")

    galvanized = model.material("galvanized_steel", rgba=(0.72, 0.74, 0.76, 1.0))
    coated_panel = model.material("coated_panel", rgba=(0.55, 0.57, 0.60, 1.0))
    gasket_dark = model.material("dark_trim", rgba=(0.18, 0.19, 0.20, 1.0))

    body = model.part("tower_body")

    def body_box(name: str, size: tuple[float, float, float], xyz: tuple[float, float, float], material=galvanized) -> None:
        body.visual(Box(size), origin=Origin(xyz=xyz), material=material, name=name)

    def body_barrel(name: str, length: float, xyz: tuple[float, float, float]) -> None:
        body.visual(
            Cylinder(radius=0.012, length=length),
            origin=Origin(xyz=xyz, rpy=(0.0, pi / 2.0, 0.0)),
            material=galvanized,
            name=name,
        )

    # Roof mounting flange and curb: one low-part-count stamped / folded base shell.
    body_box("roof_flange", (0.48, 0.38, 0.006), (0.0, 0.0, 0.003))
    body_box("front_curb", (0.36, 0.018, 0.085), (0.0, 0.131, 0.0485))
    body_box("back_curb", (0.36, 0.018, 0.085), (0.0, -0.131, 0.0485))
    body_box("left_curb", (0.018, 0.244, 0.085), (-0.171, 0.0, 0.0485))
    body_box("right_curb", (0.018, 0.244, 0.085), (0.171, 0.0, 0.0485))

    # Upper tower with framed vent faces.
    body_box("front_left_stile", (0.048, 0.014, 0.330), (-0.146, 0.123, 0.256))
    body_box("front_right_stile", (0.048, 0.014, 0.330), (0.146, 0.123, 0.256))
    body_box("front_sill", (0.244, 0.014, 0.040), (0.0, 0.123, 0.111))
    body_box("front_header", (0.244, 0.014, 0.070), (0.0, 0.123, 0.386))

    body_box("back_panel", (0.340, 0.014, 0.330), (0.0, -0.123, 0.256))

    body_box("left_side_lower_rail", (0.014, 0.246, 0.065), (-0.163, 0.0, 0.1235))
    body_box("left_side_upper_rail", (0.014, 0.246, 0.070), (-0.163, 0.0, 0.386))
    body_box("left_side_mullion", (0.014, 0.022, 0.200), (-0.163, 0.0, 0.256))

    body_box("right_side_lower_rail", (0.014, 0.246, 0.065), (0.163, 0.0, 0.1235))
    body_box("right_side_upper_rail", (0.014, 0.246, 0.070), (0.163, 0.0, 0.386))
    body_box("right_side_mullion", (0.014, 0.022, 0.200), (0.163, 0.0, 0.256))

    # Simple folded hood above the opening to read as a weathered roof vent tower.
    body_box("top_cap", (0.392, 0.302, 0.020), (0.0, -0.004, 0.428))
    body_box("front_drip_skirt", (0.392, 0.024, 0.038), (0.0, 0.139, 0.399))

    # Explicit hinge support plates and stops.
    body_box("left_hinge_plate", (0.050, 0.018, 0.038), (-0.117, 0.140, 0.362))
    body_box("right_hinge_plate", (0.050, 0.018, 0.038), (0.117, 0.140, 0.362))
    body_barrel("left_hinge_barrel", 0.050, (-0.117, 0.148, 0.362))
    body_barrel("right_hinge_barrel", 0.050, (0.117, 0.148, 0.362))

    body_box("closed_stop", (0.236, 0.014, 0.018), (0.0, 0.135, 0.139), material=gasket_dark)
    body_box("left_open_stop", (0.022, 0.022, 0.055), (-0.123, 0.117, 0.305), material=gasket_dark)
    body_box("right_open_stop", (0.022, 0.022, 0.055), (0.123, 0.117, 0.305), material=gasket_dark)

    flap = model.part("weather_flap")

    def flap_box(name: str, size: tuple[float, float, float], xyz: tuple[float, float, float], material=coated_panel) -> None:
        flap.visual(Box(size), origin=Origin(xyz=xyz), material=material, name=name)

    flap.visual(
        Cylinder(radius=0.012, length=0.180),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=coated_panel,
        name="hinge_barrel",
    )
    flap_box("top_hem", (0.292, 0.020, 0.024), (0.0, 0.025, -0.010))
    flap_box("weather_panel", (0.292, 0.010, 0.224), (0.0, 0.017, -0.124))
    flap_box("bottom_hem", (0.260, 0.024, 0.018), (0.0, 0.006, -0.223))
    flap_box("left_strap", (0.022, 0.020, 0.050), (-0.085, 0.006, -0.024))
    flap_box("right_strap", (0.022, 0.020, 0.050), (0.085, 0.006, -0.024))
    flap_box("left_return", (0.018, 0.030, 0.184), (-0.137, 0.008, -0.128))
    flap_box("right_return", (0.018, 0.030, 0.184), (0.137, 0.008, -0.128))

    model.articulation(
        "body_to_flap",
        ArticulationType.REVOLUTE,
        parent=body,
        child=flap,
        origin=Origin(xyz=(0.0, 0.148, 0.362)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.5, lower=0.0, upper=1.2),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("tower_body")
    flap = object_model.get_part("weather_flap")
    hinge = object_model.get_articulation("body_to_flap")

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

    with ctx.pose({hinge: 0.0}):
        ctx.expect_contact(
            flap,
            body,
            elem_a="bottom_hem",
            elem_b="closed_stop",
            contact_tol=0.0015,
            name="flap seats on closed stop",
        )
        ctx.expect_gap(
            flap,
            body,
            axis="y",
            positive_elem="weather_panel",
            negative_elem="front_header",
            min_gap=0.010,
            max_gap=0.035,
            name="flap sits just proud of tower frame",
        )

        panel = ctx.part_element_world_aabb(flap, elem="weather_panel")
        left = ctx.part_element_world_aabb(body, elem="front_left_stile")
        right = ctx.part_element_world_aabb(body, elem="front_right_stile")
        sill = ctx.part_element_world_aabb(body, elem="front_sill")
        header = ctx.part_element_world_aabb(body, elem="front_header")

        covers_opening = False
        details = "missing AABB for flap coverage check"
        if panel and left and right and sill and header:
            opening_min_x = left[1][0]
            opening_max_x = right[0][0]
            opening_min_z = sill[1][2]
            opening_max_z = header[0][2]
            panel_min_x = panel[0][0]
            panel_max_x = panel[1][0]
            panel_min_z = panel[0][2]
            panel_max_z = panel[1][2]
            covers_opening = (
                panel_min_x <= opening_min_x - 0.008
                and panel_max_x >= opening_max_x + 0.008
                and panel_min_z <= opening_min_z + 0.004
                and panel_max_z >= opening_max_z - 0.008
            )
            details = (
                f"panel x=({panel_min_x:.3f},{panel_max_x:.3f}) vs opening x=({opening_min_x:.3f},{opening_max_x:.3f}); "
                f"panel z=({panel_min_z:.3f},{panel_max_z:.3f}) vs opening z=({opening_min_z:.3f},{opening_max_z:.3f})"
            )
        ctx.check("flap covers framed opening when closed", covers_opening, details)

        closed_bottom = ctx.part_element_world_aabb(flap, elem="bottom_hem")

    with ctx.pose({hinge: 1.1}):
        ctx.fail_if_parts_overlap_in_current_pose(name="no overlap with flap open")
        open_bottom = ctx.part_element_world_aabb(flap, elem="bottom_hem")

    flap_lifts_cleanly = False
    lift_details = "missing bottom hem AABB for open/closed comparison"
    if closed_bottom and open_bottom:
        closed_max_y = closed_bottom[1][1]
        closed_min_z = closed_bottom[0][2]
        open_max_y = open_bottom[1][1]
        open_min_z = open_bottom[0][2]
        flap_lifts_cleanly = (
            open_max_y >= closed_max_y + 0.090
            and open_min_z >= closed_min_z + 0.090
        )
        lift_details = (
            f"closed bottom hem max_y={closed_max_y:.3f}, min_z={closed_min_z:.3f}; "
            f"open max_y={open_max_y:.3f}, min_z={open_min_z:.3f}"
        )
    ctx.check("flap opens upward and outward", flap_lifts_cleanly, lift_details)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
