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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="roof_vent_tower")

    galvanized = model.material("galvanized", rgba=(0.63, 0.65, 0.67, 1.0))
    hardware = model.material("hardware", rgba=(0.30, 0.32, 0.35, 1.0))
    seal = model.material("seal", rgba=(0.11, 0.11, 0.12, 1.0))
    wear = model.material("wear_strip", rgba=(0.82, 0.58, 0.16, 1.0))

    curb_base = model.part("curb_base")
    curb_base.visual(
        Box((1.02, 0.92, 0.02)),
        origin=Origin(xyz=(0.0, 0.0, 0.01)),
        material=hardware,
        name="roof_flange",
    )
    curb_base.visual(
        Box((0.90, 0.05, 0.18)),
        origin=Origin(xyz=(0.0, 0.375, 0.11)),
        material=galvanized,
        name="curb_front_wall",
    )
    curb_base.visual(
        Box((0.90, 0.05, 0.18)),
        origin=Origin(xyz=(0.0, -0.375, 0.11)),
        material=galvanized,
        name="curb_back_wall",
    )
    curb_base.visual(
        Box((0.05, 0.70, 0.18)),
        origin=Origin(xyz=(-0.425, 0.0, 0.11)),
        material=galvanized,
        name="curb_left_wall",
    )
    curb_base.visual(
        Box((0.05, 0.70, 0.18)),
        origin=Origin(xyz=(0.425, 0.0, 0.11)),
        material=galvanized,
        name="curb_right_wall",
    )

    tower_frame = model.part("tower_frame")
    for name, x, y in (
        ("post_front_left", -0.38, 0.33),
        ("post_front_right", 0.38, 0.33),
        ("post_back_left", -0.38, -0.33),
        ("post_back_right", 0.38, -0.33),
    ):
        tower_frame.visual(
            Box((0.08, 0.08, 0.98)),
            origin=Origin(xyz=(x, y, 0.49)),
            material=galvanized,
            name=name,
        )

    for name, y in (("lower_rear_rail", -0.34), ("upper_rear_rail", -0.34)):
        tower_frame.visual(
            Box((0.68, 0.06, 0.06)),
            origin=Origin(xyz=(0.0, y, 0.12 if "lower" in name else 0.92)),
            material=galvanized,
            name=name,
        )
    tower_frame.visual(
        Box((0.68, 0.08, 0.06)),
        origin=Origin(xyz=(0.0, 0.34, 0.11)),
        material=galvanized,
        name="front_sill",
    )
    tower_frame.visual(
        Box((0.68, 0.06, 0.08)),
        origin=Origin(xyz=(0.0, 0.338, 0.72)),
        material=galvanized,
        name="front_header",
    )
    tower_frame.visual(
        Box((0.68, 0.06, 0.06)),
        origin=Origin(xyz=(0.0, 0.34, 0.92)),
        material=galvanized,
        name="upper_front_rail",
    )
    tower_frame.visual(
        Box((0.06, 0.58, 0.06)),
        origin=Origin(xyz=(-0.39, 0.0, 0.12)),
        material=galvanized,
        name="lower_left_rail",
    )
    tower_frame.visual(
        Box((0.06, 0.58, 0.06)),
        origin=Origin(xyz=(0.39, 0.0, 0.12)),
        material=galvanized,
        name="lower_right_rail",
    )
    tower_frame.visual(
        Box((0.06, 0.58, 0.06)),
        origin=Origin(xyz=(-0.39, 0.0, 0.92)),
        material=galvanized,
        name="upper_left_rail",
    )
    tower_frame.visual(
        Box((0.06, 0.58, 0.06)),
        origin=Origin(xyz=(0.39, 0.0, 0.92)),
        material=galvanized,
        name="upper_right_rail",
    )
    tower_frame.visual(
        Box((0.06, 0.06, 0.54)),
        origin=Origin(xyz=(-0.31, 0.338, 0.41)),
        material=galvanized,
        name="front_left_jamb",
    )
    tower_frame.visual(
        Box((0.06, 0.06, 0.54)),
        origin=Origin(xyz=(0.31, 0.338, 0.41)),
        material=galvanized,
        name="front_right_jamb",
    )

    for i, z in enumerate((0.27, 0.42, 0.57, 0.72), start=1):
        tower_frame.visual(
            Box((0.03, 0.64, 0.03)),
            origin=Origin(xyz=(-0.405, 0.0, z), rpy=(math.radians(24.0), 0.0, 0.0)),
            material=galvanized,
            name=f"left_louver_{i}",
        )
        tower_frame.visual(
            Box((0.03, 0.64, 0.03)),
            origin=Origin(xyz=(0.405, 0.0, z), rpy=(math.radians(-24.0), 0.0, 0.0)),
            material=galvanized,
            name=f"right_louver_{i}",
        )
        tower_frame.visual(
            Box((0.74, 0.03, 0.03)),
            origin=Origin(xyz=(0.0, -0.355, z), rpy=(0.0, math.radians(24.0), 0.0)),
            material=galvanized,
            name=f"rear_louver_{i}",
        )

    tower_frame.visual(
        Box((0.018, 0.006, 0.52)),
        origin=Origin(xyz=(-0.321, 0.373, 0.41)),
        material=seal,
        name="gasket_left",
    )
    tower_frame.visual(
        Box((0.018, 0.006, 0.52)),
        origin=Origin(xyz=(0.321, 0.373, 0.41)),
        material=seal,
        name="gasket_right",
    )
    tower_frame.visual(
        Box((0.64, 0.006, 0.018)),
        origin=Origin(xyz=(0.0, 0.373, 0.681)),
        material=seal,
        name="gasket_top",
    )
    tower_frame.visual(
        Box((0.64, 0.006, 0.018)),
        origin=Origin(xyz=(0.0, 0.373, 0.141)),
        material=seal,
        name="gasket_bottom",
    )

    for name, x in (("hinge_plate_left", -0.27), ("hinge_plate_center", 0.0), ("hinge_plate_right", 0.27)):
        tower_frame.visual(
            Box((0.11, 0.022, 0.14)),
            origin=Origin(xyz=(x, 0.377, 0.75)),
            material=hardware,
            name=name,
        )

    for name, x in (("tower_barrel_left", -0.27), ("tower_barrel_center", 0.0), ("tower_barrel_right", 0.27)):
        tower_frame.visual(
            Cylinder(radius=0.025, length=0.10),
            origin=Origin(xyz=(x, 0.399, 0.70), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=hardware,
            name=name,
        )

    for side, x in (("left", -0.37), ("right", 0.37)):
        tower_frame.visual(
            Box((0.04, 0.08, 0.16)),
            origin=Origin(xyz=(x, 0.36, 0.72)),
            material=hardware,
            name=f"stop_bracket_{side}",
        )
        tower_frame.visual(
            Box((0.04, 0.04, 0.10)),
            origin=Origin(xyz=(x, 0.408, 0.64)),
            material=hardware,
            name=f"open_stop_{side}",
        )

    tower_frame.visual(
        Box((0.94, 0.84, 0.04)),
        origin=Origin(xyz=(0.0, 0.0, 0.96)),
        material=galvanized,
        name="roof_cap",
    )
    tower_frame.visual(
        Box((0.94, 0.05, 0.03)),
        origin=Origin(xyz=(0.0, 0.395, 0.925)),
        material=galvanized,
        name="roof_drip_edge",
    )

    flap = model.part("flap")
    flap.visual(
        Box((0.70, 0.055, 0.05)),
        origin=Origin(xyz=(0.0, 0.0045, -0.065)),
        material=galvanized,
        name="top_rail",
    )
    flap.visual(
        Box((0.70, 0.055, 0.05)),
        origin=Origin(xyz=(0.0, 0.0045, -0.525)),
        material=galvanized,
        name="bottom_rail",
    )
    flap.visual(
        Box((0.05, 0.055, 0.49)),
        origin=Origin(xyz=(-0.325, 0.0045, -0.272)),
        material=galvanized,
        name="left_stile",
    )
    flap.visual(
        Box((0.05, 0.055, 0.49)),
        origin=Origin(xyz=(0.325, 0.0045, -0.272)),
        material=galvanized,
        name="right_stile",
    )
    flap.visual(
        Box((0.60, 0.008, 0.44)),
        origin=Origin(xyz=(0.0, 0.028, -0.272)),
        material=galvanized,
        name="skin_panel",
    )
    flap.visual(
        Box((0.04, 0.024, 0.42)),
        origin=Origin(xyz=(-0.17, 0.012, -0.282)),
        material=hardware,
        name="rib_left",
    )
    flap.visual(
        Box((0.04, 0.024, 0.42)),
        origin=Origin(xyz=(0.17, 0.012, -0.282)),
        material=hardware,
        name="rib_right",
    )
    flap.visual(
        Box((0.16, 0.014, 0.08)),
        origin=Origin(xyz=(-0.135, 0.0, -0.04)),
        material=hardware,
        name="leaf_left",
    )
    flap.visual(
        Box((0.16, 0.014, 0.08)),
        origin=Origin(xyz=(0.135, 0.0, -0.04)),
        material=hardware,
        name="leaf_right",
    )
    flap.visual(
        Cylinder(radius=0.023, length=0.14),
        origin=Origin(xyz=(-0.135, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hardware,
        name="flap_barrel_left",
    )
    flap.visual(
        Cylinder(radius=0.023, length=0.14),
        origin=Origin(xyz=(0.135, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hardware,
        name="flap_barrel_right",
    )
    flap.visual(
        Box((0.03, 0.016, 0.09)),
        origin=Origin(xyz=(-0.335, 0.0, -0.02)),
        material=hardware,
        name="stop_ear_left",
    )
    flap.visual(
        Box((0.03, 0.016, 0.09)),
        origin=Origin(xyz=(0.335, 0.0, -0.02)),
        material=hardware,
        name="stop_ear_right",
    )
    flap.visual(
        Box((0.64, 0.04, 0.016)),
        origin=Origin(xyz=(0.0, 0.008, -0.552)),
        material=wear,
        name="wear_strip",
    )
    flap.visual(
        Box((0.02, 0.04, 0.08)),
        origin=Origin(xyz=(-0.12, 0.052, -0.29)),
        material=hardware,
        name="handle_standoff_left",
    )
    flap.visual(
        Box((0.02, 0.04, 0.08)),
        origin=Origin(xyz=(0.12, 0.052, -0.29)),
        material=hardware,
        name="handle_standoff_right",
    )
    flap.visual(
        Cylinder(radius=0.012, length=0.30),
        origin=Origin(xyz=(0.0, 0.072, -0.29), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hardware,
        name="service_handle",
    )

    model.articulation(
        "curb_to_tower",
        ArticulationType.FIXED,
        parent=curb_base,
        child=tower_frame,
        origin=Origin(xyz=(0.0, 0.0, 0.20)),
    )
    model.articulation(
        "tower_to_flap",
        ArticulationType.REVOLUTE,
        parent=tower_frame,
        child=flap,
        origin=Origin(xyz=(0.0, 0.399, 0.70)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=35.0,
            velocity=1.2,
            lower=0.0,
            upper=1.2,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    curb_base = object_model.get_part("curb_base")
    tower_frame = object_model.get_part("tower_frame")
    flap = object_model.get_part("flap")
    flap_hinge = object_model.get_articulation("tower_to_flap")

    def _aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]]) -> tuple[float, float, float]:
        return tuple((lo + hi) * 0.5 for lo, hi in zip(aabb[0], aabb[1]))

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
        "parts_present",
        all(part is not None for part in (curb_base, tower_frame, flap)),
        "Expected curb_base, tower_frame, and flap parts to exist.",
    )
    ctx.check(
        "flap_hinge_axis_and_limits",
        flap_hinge.axis == (1.0, 0.0, 0.0)
        and flap_hinge.motion_limits is not None
        and flap_hinge.motion_limits.lower == 0.0
        and flap_hinge.motion_limits.upper is not None
        and flap_hinge.motion_limits.upper >= 1.1,
        "Flap hinge must use an X-axis hinge line with a serviceable open angle.",
    )
    ctx.expect_contact(curb_base, tower_frame, name="tower_seats_on_curb")

    with ctx.pose({flap_hinge: 0.0}):
        ctx.expect_contact(
            flap,
            tower_frame,
            elem_a="left_stile",
            elem_b="gasket_left",
            name="left_flap_edge_seats_on_gasket",
        )
        ctx.expect_contact(
            flap,
            tower_frame,
            elem_a="bottom_rail",
            elem_b="gasket_bottom",
            name="bottom_rail_seats_on_sill_gasket",
        )
        ctx.expect_overlap(
            flap,
            tower_frame,
            axes="xz",
            min_overlap=0.45,
            name="closed_flap_covers_service_opening",
        )

    with ctx.pose({flap_hinge: 0.0}):
        closed_wear = ctx.part_element_world_aabb(flap, elem="wear_strip")
    with ctx.pose({flap_hinge: 1.05}):
        open_wear = ctx.part_element_world_aabb(flap, elem="wear_strip")
        ctx.expect_gap(
            flap,
            tower_frame,
            axis="y",
            positive_elem="wear_strip",
            min_gap=0.10,
            name="open_flap_wear_strip_clears_tower_front",
        )

    if closed_wear is None or open_wear is None:
        ctx.fail("wear_strip_pose_tracking", "Could not resolve wear_strip AABBs for open/closed poses.")
    else:
        closed_center = _aabb_center(closed_wear)
        open_center = _aabb_center(open_wear)
        ctx.check(
            "flap_opens_outward_and_up",
            open_center[1] > closed_center[1] + 0.20 and open_center[2] > closed_center[2] + 0.10,
            (
                "Expected wear strip center to move outward and upward when the flap opens; "
                f"closed={closed_center}, open={open_center}."
            ),
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
