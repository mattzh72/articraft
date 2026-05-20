from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)

ASSETS = AssetContext.from_script(__file__)


def _write_knob_mesh():
    profile = [
        (0.0, -0.014),
        (0.011, -0.014),
        (0.015, -0.012),
        (0.0175, -0.008),
        (0.0185, -0.001),
        (0.0185, 0.008),
        (0.0170, 0.0115),
        (0.0140, 0.0135),
        (0.0, 0.014),
    ]
    return mesh_from_geometry(
        LatheGeometry(profile, segments=40, closed=True),
        ASSETS.mesh_path("cooktop_knob.obj"),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="open_shelf_gas_cooktop", assets=ASSETS)

    oak = model.material("oak", rgba=(0.72, 0.56, 0.38, 1.0))
    stainless = model.material("stainless", rgba=(0.78, 0.80, 0.82, 1.0))
    cast_iron = model.material("cast_iron", rgba=(0.13, 0.13, 0.14, 1.0))
    control_black = model.material("control_black", rgba=(0.10, 0.10, 0.11, 1.0))
    burner_dark = model.material("burner_dark", rgba=(0.18, 0.18, 0.19, 1.0))
    brushed_aluminum = model.material("brushed_aluminum", rgba=(0.70, 0.71, 0.73, 1.0))

    shelf_unit = model.part("shelf_unit")
    shelf_unit.visual(
        Box((0.03, 0.50, 0.86)),
        origin=Origin(xyz=(-0.585, 0.0, 0.43)),
        material=oak,
        name="left_panel",
    )
    shelf_unit.visual(
        Box((0.03, 0.50, 0.86)),
        origin=Origin(xyz=(0.585, 0.0, 0.43)),
        material=oak,
        name="right_panel",
    )
    shelf_unit.visual(
        Box((1.14, 0.46, 0.03)),
        origin=Origin(xyz=(0.0, 0.0, 0.115)),
        material=oak,
        name="bottom_shelf",
    )
    shelf_unit.visual(
        Box((1.14, 0.46, 0.03)),
        origin=Origin(xyz=(0.0, 0.0, 0.435)),
        material=oak,
        name="middle_shelf",
    )
    shelf_unit.visual(
        Box((0.24, 0.31, 0.04)),
        origin=Origin(xyz=(-0.48, 0.025, 0.88)),
        material=oak,
        name="top_left_strip",
    )
    shelf_unit.visual(
        Box((0.24, 0.31, 0.04)),
        origin=Origin(xyz=(0.48, 0.025, 0.88)),
        material=oak,
        name="top_right_strip",
    )
    shelf_unit.visual(
        Box((1.20, 0.07, 0.04)),
        origin=Origin(xyz=(0.0, 0.215, 0.88)),
        material=oak,
        name="top_rear_strip",
    )
    shelf_unit.inertial = Inertial.from_geometry(
        Box((1.20, 0.50, 0.90)),
        mass=38.0,
        origin=Origin(xyz=(0.0, 0.0, 0.45)),
    )

    cooktop = model.part("cooktop")
    cooktop.visual(
        Box((0.760, 0.440, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material=stainless,
        name="top_plate",
    )
    cooktop.visual(
        Box((0.760, 0.010, 0.040)),
        origin=Origin(xyz=(0.0, -0.215, -0.020)),
        material=stainless,
        name="front_fascia",
    )
    cooktop.visual(
        Box((0.004, 0.390, 0.068)),
        origin=Origin(xyz=(-0.353, 0.0, -0.034)),
        material=stainless,
        name="tray_left_wall",
    )
    cooktop.visual(
        Box((0.004, 0.390, 0.068)),
        origin=Origin(xyz=(0.353, 0.0, -0.034)),
        material=stainless,
        name="tray_right_wall",
    )
    cooktop.visual(
        Box((0.702, 0.004, 0.068)),
        origin=Origin(xyz=(0.0, -0.193, -0.034)),
        material=stainless,
        name="tray_front_wall",
    )
    cooktop.visual(
        Box((0.702, 0.004, 0.068)),
        origin=Origin(xyz=(0.0, 0.193, -0.034)),
        material=stainless,
        name="tray_rear_wall",
    )
    cooktop.visual(
        Box((0.702, 0.386, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, -0.066)),
        material=stainless,
        name="tray_bottom",
    )

    burner_specs = {
        "rear_left": {"xy": (-0.17, 0.10), "base": 0.042, "ring": 0.032, "cap": 0.024},
        "front_left": {"xy": (-0.17, -0.10), "base": 0.048, "ring": 0.036, "cap": 0.026},
        "rear_right": {"xy": (0.17, 0.10), "base": 0.036, "ring": 0.027, "cap": 0.020},
        "front_right": {"xy": (0.17, -0.10), "base": 0.040, "ring": 0.030, "cap": 0.022},
    }
    for burner_name, spec in burner_specs.items():
        bx, by = spec["xy"]
        cooktop.visual(
            Cylinder(radius=spec["base"], length=0.008),
            origin=Origin(xyz=(bx, by, 0.008)),
            material=burner_dark,
            name=f"{burner_name}_base",
        )
        cooktop.visual(
            Cylinder(radius=spec["ring"], length=0.006),
            origin=Origin(xyz=(bx, by, 0.015)),
            material=burner_dark,
            name=f"{burner_name}_ring",
        )
        cooktop.visual(
            Cylinder(radius=spec["cap"], length=0.010),
            origin=Origin(xyz=(bx, by, 0.023)),
            material=cast_iron,
            name=f"{burner_name}_cap",
        )

    cooktop.inertial = Inertial.from_geometry(
        Box((0.760, 0.440, 0.098)),
        mass=11.0,
        origin=Origin(xyz=(0.0, 0.0, -0.021)),
    )

    model.articulation(
        "shelf_unit_to_cooktop",
        ArticulationType.FIXED,
        parent=shelf_unit,
        child=cooktop,
        origin=Origin(xyz=(0.0, -0.035, 0.90)),
    )

    def add_grate(
        name: str,
        *,
        x: float,
        y: float,
        width: float,
        depth: float,
    ) -> None:
        grate = model.part(name)
        rail_w = 0.012
        rail_h = 0.012
        foot_size = 0.016
        foot_height = 0.021
        top_z = 0.031
        foot_z = foot_height / 2.0 + 0.004

        grate.visual(
            Box((rail_w, depth, rail_h)),
            origin=Origin(xyz=(-width / 2.0 + rail_w / 2.0, 0.0, top_z)),
            material=cast_iron,
            name="left_rail",
        )
        grate.visual(
            Box((rail_w, depth, rail_h)),
            origin=Origin(xyz=(width / 2.0 - rail_w / 2.0, 0.0, top_z)),
            material=cast_iron,
            name="right_rail",
        )
        grate.visual(
            Box((width, rail_w, rail_h)),
            origin=Origin(xyz=(0.0, -depth / 2.0 + rail_w / 2.0, top_z)),
            material=cast_iron,
            name="front_rail",
        )
        grate.visual(
            Box((width, rail_w, rail_h)),
            origin=Origin(xyz=(0.0, depth / 2.0 - rail_w / 2.0, top_z)),
            material=cast_iron,
            name="rear_rail",
        )
        grate.visual(
            Box((width, rail_w, 0.010)),
            origin=Origin(xyz=(0.0, 0.0, top_z)),
            material=cast_iron,
            name="cross_x",
        )
        grate.visual(
            Box((rail_w, depth, 0.010)),
            origin=Origin(xyz=(0.0, 0.0, top_z)),
            material=cast_iron,
            name="cross_y",
        )

        foot_x = width / 2.0 - rail_w / 2.0
        foot_y = depth / 2.0 - rail_w / 2.0
        for foot_name, local_x, local_y in (
            ("front_left_foot", -foot_x, -foot_y),
            ("front_right_foot", foot_x, -foot_y),
            ("rear_left_foot", -foot_x, foot_y),
            ("rear_right_foot", foot_x, foot_y),
        ):
            grate.visual(
                Box((foot_size, foot_size, foot_height)),
                origin=Origin(xyz=(local_x, local_y, foot_z)),
                material=cast_iron,
                name=foot_name,
            )

        grate.inertial = Inertial.from_geometry(
            Box((width, depth, 0.033)),
            mass=1.15,
            origin=Origin(xyz=(0.0, 0.0, 0.019)),
        )

        model.articulation(
            f"cooktop_to_{name}",
            ArticulationType.FIXED,
            parent=cooktop,
            child=grate,
            origin=Origin(xyz=(x, y, 0.0)),
        )

    add_grate("grate_rear_left", x=-0.17, y=0.10, width=0.180, depth=0.180)
    add_grate("grate_front_left", x=-0.17, y=-0.10, width=0.190, depth=0.190)
    add_grate("grate_rear_right", x=0.17, y=0.10, width=0.170, depth=0.170)
    add_grate("grate_front_right", x=0.17, y=-0.10, width=0.175, depth=0.175)

    knob_mesh = _write_knob_mesh()

    def add_knob(name: str, *, x: float) -> None:
        knob = model.part(name)
        knob.visual(
            knob_mesh,
            origin=Origin(xyz=(0.0, -0.014, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=control_black,
            name="body",
        )
        knob.visual(
            Cylinder(radius=0.0055, length=0.010),
            origin=Origin(xyz=(0.0, -0.005, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=burner_dark,
            name="spindle",
        )
        knob.visual(
            Box((0.004, 0.017, 0.016)),
            origin=Origin(xyz=(0.0, -0.026, 0.0)),
            material=brushed_aluminum,
            name="grip_inlay",
        )
        knob.visual(
            Box((0.010, 0.003, 0.004)),
            origin=Origin(xyz=(0.0, -0.028, 0.0145)),
            material=stainless,
            name="indicator",
        )
        knob.inertial = Inertial.from_geometry(
            Box((0.036, 0.032, 0.036)),
            mass=0.18,
            origin=Origin(xyz=(0.0, -0.016, 0.0)),
        )

        model.articulation(
            f"cooktop_to_{name}",
            ArticulationType.CONTINUOUS,
            parent=cooktop,
            child=knob,
            origin=Origin(xyz=(x, -0.220, -0.020)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=0.4, velocity=8.0),
        )

    add_knob("knob_front_left_outer", x=-0.305)
    add_knob("knob_front_left_inner", x=-0.235)
    add_knob("knob_front_right_inner", x=0.235)
    add_knob("knob_front_right_outer", x=0.305)

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    shelf_unit = object_model.get_part("shelf_unit")
    cooktop = object_model.get_part("cooktop")
    grates = [
        object_model.get_part("grate_rear_left"),
        object_model.get_part("grate_front_left"),
        object_model.get_part("grate_rear_right"),
        object_model.get_part("grate_front_right"),
    ]
    knob_parts = [
        object_model.get_part("knob_front_left_outer"),
        object_model.get_part("knob_front_left_inner"),
        object_model.get_part("knob_front_right_inner"),
        object_model.get_part("knob_front_right_outer"),
    ]
    knob_joint_names = [
        "cooktop_to_knob_front_left_outer",
        "cooktop_to_knob_front_left_inner",
        "cooktop_to_knob_front_right_inner",
        "cooktop_to_knob_front_right_outer",
    ]
    knob_joints = [object_model.get_articulation(name) for name in knob_joint_names]

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts(max_pose_samples=12)
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
    ctx.fail_if_articulation_overlaps(max_pose_samples=24)

    ctx.expect_contact(cooktop, shelf_unit, name="cooktop_supported_by_worktop")
    ctx.expect_contact(
        cooktop,
        shelf_unit,
        elem_a="top_plate",
        elem_b="top_left_strip",
        name="cooktop_lip_contacts_left_strip",
    )
    ctx.expect_contact(
        cooktop,
        shelf_unit,
        elem_a="top_plate",
        elem_b="top_right_strip",
        name="cooktop_lip_contacts_right_strip",
    )
    ctx.expect_contact(
        cooktop,
        shelf_unit,
        elem_a="top_plate",
        elem_b="top_rear_strip",
        name="cooktop_lip_contacts_rear_strip",
    )

    for grate, burner_name in zip(
        grates,
        ("rear_left", "front_left", "rear_right", "front_right"),
    ):
        ctx.expect_contact(grate, cooktop, name=f"{grate.name}_rests_on_cooktop")
        ctx.expect_overlap(
            grate,
            cooktop,
            axes="xy",
            elem_b=f"{burner_name}_cap",
            min_overlap=0.035,
            name=f"{grate.name}_centered_over_{burner_name}_burner",
        )

    for knob in knob_parts:
        ctx.expect_contact(knob, cooktop, name=f"{knob.name}_mounted_to_fascia")

    moving_joints = [
        articulation
        for articulation in object_model.articulations
        if articulation.articulation_type != ArticulationType.FIXED
    ]
    ctx.check(
        "only_four_moving_articulations",
        len(moving_joints) == 4,
        details=f"Expected 4 moving joints, found {len(moving_joints)}.",
    )

    for joint in knob_joints:
        limits = joint.motion_limits
        joint_is_continuous = joint.articulation_type == ArticulationType.CONTINUOUS
        axis_ok = tuple(joint.axis) == (0.0, 1.0, 0.0)
        limits_ok = limits is not None and limits.lower is None and limits.upper is None
        ctx.check(
            f"{joint.name}_is_continuous",
            joint_is_continuous,
            details="Knob joints must be continuous articulations.",
        )
        ctx.check(
            f"{joint.name}_axis_front_to_back",
            axis_ok,
            details=f"Expected axis (0, 1, 0), got {joint.axis}.",
        )
        ctx.check(
            f"{joint.name}_unbounded_limits",
            limits_ok,
            details="Continuous knob joints must omit lower and upper limits.",
        )

    knob_positions = {knob.name: ctx.part_world_position(knob) for knob in knob_parts}
    if any(position is None for position in knob_positions.values()):
        ctx.fail("knob_positions_available", "All knob part positions should resolve in world space.")
    else:
        left_outer = knob_positions["knob_front_left_outer"]
        left_inner = knob_positions["knob_front_left_inner"]
        right_inner = knob_positions["knob_front_right_inner"]
        right_outer = knob_positions["knob_front_right_outer"]
        assert left_outer is not None
        assert left_inner is not None
        assert right_inner is not None
        assert right_outer is not None

        center_gap = right_inner[0] - left_inner[0]
        same_row = max(
            abs(left_outer[1] - left_inner[1]),
            abs(left_inner[1] - right_inner[1]),
            abs(right_inner[1] - right_outer[1]),
        )
        ctx.check(
            "knobs_leave_center_lip_clear",
            center_gap >= 0.40,
            details=f"Expected a clear center zone at least 0.40 m wide, got {center_gap:.3f} m.",
        )
        ctx.check(
            "left_knob_pair_stays_left",
            left_inner[0] < -0.15 and left_outer[0] < -0.20,
            details="Left knob pair should cluster at the front-left corner.",
        )
        ctx.check(
            "right_knob_pair_stays_right",
            right_inner[0] > 0.15 and right_outer[0] > 0.20,
            details="Right knob pair should cluster at the front-right corner.",
        )
        ctx.check(
            "all_knobs_share_front_row",
            same_row <= 1e-6,
            details=f"Knob y positions diverged by {same_row:.6f} m.",
        )

    indicator_rest = ctx.part_element_world_aabb(knob_parts[0], elem="indicator")
    if indicator_rest is None:
        ctx.fail("knob_indicator_present", "Knob indicator visual should resolve for articulation checks.")
    else:
        rest_center = (
            (indicator_rest[0][0] + indicator_rest[1][0]) / 2.0,
            (indicator_rest[0][2] + indicator_rest[1][2]) / 2.0,
        )
        with ctx.pose({knob_joints[0]: math.pi / 2.0}):
            indicator_turn = ctx.part_element_world_aabb(knob_parts[0], elem="indicator")
            if indicator_turn is None:
                ctx.fail("knob_indicator_turn_pose", "Indicator AABB should resolve in turned pose.")
            else:
                turn_center = (
                    (indicator_turn[0][0] + indicator_turn[1][0]) / 2.0,
                    (indicator_turn[0][2] + indicator_turn[1][2]) / 2.0,
                )
                ctx.check(
                    "knob_indicator_rotates_with_joint",
                    turn_center[0] > rest_center[0] + 0.010 and turn_center[1] < rest_center[1] - 0.010,
                    details=(
                        "Indicator should move from top to side after a quarter turn. "
                        f"Rest center={rest_center}, turned center={turn_center}."
                    ),
                )
                ctx.expect_contact(
                    knob_parts[0],
                    cooktop,
                    name="turned_knob_remains_mounted",
                )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
