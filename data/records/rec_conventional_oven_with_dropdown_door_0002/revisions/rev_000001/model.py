from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math
import os

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)

try:
    os.getcwd()
except FileNotFoundError:
    os.chdir("/")


def _build_gas_range() -> ArticulatedObject:
    model = ArticulatedObject(name="gas_range")

    body_enamel = model.material("body_enamel", rgba=(0.93, 0.93, 0.91, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.20, 0.21, 0.23, 1.0))
    grate_iron = model.material("grate_iron", rgba=(0.14, 0.14, 0.15, 1.0))
    knob_metal = model.material("knob_metal", rgba=(0.74, 0.75, 0.77, 1.0))
    glass = model.material("glass", rgba=(0.58, 0.71, 0.83, 0.30))
    cavity_dark = model.material("cavity_dark", rgba=(0.08, 0.08, 0.09, 1.0))

    body = model.part("body")
    body.visual(
        Box((0.030, 0.640, 0.830)),
        origin=Origin(xyz=(-0.365, 0.000, 0.415)),
        material=body_enamel,
        name="left_side",
    )
    body.visual(
        Box((0.030, 0.640, 0.830)),
        origin=Origin(xyz=(0.365, 0.000, 0.415)),
        material=body_enamel,
        name="right_side",
    )
    body.visual(
        Box((0.700, 0.030, 0.830)),
        origin=Origin(xyz=(0.000, -0.305, 0.415)),
        material=body_enamel,
        name="back_panel",
    )
    body.visual(
        Box((0.780, 0.640, 0.030)),
        origin=Origin(xyz=(0.000, 0.000, 0.845)),
        material=body_enamel,
        name="cooktop",
    )
    body.visual(
        Box((0.780, 0.050, 0.180)),
        origin=Origin(xyz=(0.000, -0.290, 0.950)),
        material=body_enamel,
        name="backsplash",
    )
    body.visual(
        Box((0.700, 0.040, 0.100)),
        origin=Origin(xyz=(0.000, -0.245, 0.910)),
        material=body_enamel,
        name="control_panel",
    )
    body.visual(
        Box((0.700, 0.068, 0.140)),
        origin=Origin(xyz=(0.000, 0.254, 0.750)),
        material=body_enamel,
        name="upper_front_panel",
    )
    body.visual(
        Box((0.760, 0.200, 0.100)),
        origin=Origin(xyz=(0.000, 0.140, 0.050)),
        material=body_enamel,
        name="toe_kick",
    )
    body.visual(
        Box((0.700, 0.034, 0.010)),
        origin=Origin(xyz=(0.000, 0.271, 0.095)),
        material=body_enamel,
        name="hinge_sill",
    )
    body.visual(
        Box((0.660, 0.034, 0.030)),
        origin=Origin(xyz=(0.000, 0.271, 0.685)),
        material=body_enamel,
        name="front_reveal_top",
    )
    body.visual(
        Box((0.030, 0.034, 0.580)),
        origin=Origin(xyz=(-0.345, 0.271, 0.390)),
        material=body_enamel,
        name="front_reveal_left",
    )
    body.visual(
        Box((0.030, 0.034, 0.580)),
        origin=Origin(xyz=(0.345, 0.271, 0.390)),
        material=body_enamel,
        name="front_reveal_right",
    )
    body.visual(
        Box((0.660, 0.540, 0.020)),
        origin=Origin(xyz=(0.000, -0.020, 0.110)),
        material=cavity_dark,
        name="oven_floor",
    )
    body.visual(
        Box((0.660, 0.540, 0.020)),
        origin=Origin(xyz=(0.000, -0.020, 0.690)),
        material=cavity_dark,
        name="oven_ceiling",
    )
    body.visual(
        Box((0.660, 0.040, 0.560)),
        origin=Origin(xyz=(0.000, -0.285, 0.400)),
        material=cavity_dark,
        name="oven_back_wall",
    )
    body.visual(
        Box((0.020, 0.540, 0.560)),
        origin=Origin(xyz=(-0.340, -0.020, 0.400)),
        material=cavity_dark,
        name="oven_left_wall",
    )
    body.visual(
        Box((0.020, 0.540, 0.560)),
        origin=Origin(xyz=(0.340, -0.020, 0.400)),
        material=cavity_dark,
        name="oven_right_wall",
    )
    body.inertial = Inertial.from_geometry(
        Box((0.780, 0.640, 1.040)),
        mass=68.0,
        origin=Origin(xyz=(0.000, 0.000, 0.520)),
    )

    burner_positions = {
        "front_left": (-0.185, 0.125),
        "front_right": (0.185, 0.125),
        "rear_left": (-0.185, -0.125),
        "rear_right": (0.185, -0.125),
    }
    for burner_name, (x_pos, y_pos) in burner_positions.items():
        burner = model.part(f"{burner_name}_burner")
        burner.visual(
            Cylinder(radius=0.072, length=0.004),
            origin=Origin(xyz=(0.000, 0.000, 0.002)),
            material=dark_steel,
            name="burner_ring",
        )
        burner.visual(
            Cylinder(radius=0.044, length=0.012),
            origin=Origin(xyz=(0.000, 0.000, 0.010)),
            material=dark_steel,
            name="burner_cap",
        )
        burner.inertial = Inertial.from_geometry(
            Cylinder(radius=0.072, length=0.020),
            mass=0.45,
            origin=Origin(xyz=(0.000, 0.000, 0.010)),
        )
        model.articulation(
            f"body_to_{burner_name}_burner",
            ArticulationType.FIXED,
            parent=body,
            child=burner,
            origin=Origin(xyz=(x_pos, y_pos, 0.860)),
        )

        grate = model.part(f"{burner_name}_grate")
        grate.visual(
            Box((0.180, 0.014, 0.006)),
            origin=Origin(xyz=(0.000, -0.070, 0.015)),
            material=grate_iron,
            name="front_bar",
        )
        grate.visual(
            Box((0.180, 0.014, 0.006)),
            origin=Origin(xyz=(0.000, 0.070, 0.015)),
            material=grate_iron,
            name="rear_bar",
        )
        grate.visual(
            Box((0.014, 0.180, 0.006)),
            origin=Origin(xyz=(-0.070, 0.000, 0.015)),
            material=grate_iron,
            name="left_bar",
        )
        grate.visual(
            Box((0.014, 0.180, 0.006)),
            origin=Origin(xyz=(0.070, 0.000, 0.015)),
            material=grate_iron,
            name="right_bar",
        )
        grate.visual(
            Box((0.180, 0.014, 0.006)),
            origin=Origin(xyz=(0.000, 0.000, 0.021)),
            material=grate_iron,
            name="center_bar_x",
        )
        grate.visual(
            Box((0.014, 0.180, 0.006)),
            origin=Origin(xyz=(0.000, 0.000, 0.021)),
            material=grate_iron,
            name="center_bar_y",
        )
        for foot_index, (foot_x, foot_y) in enumerate(
            ((-0.070, -0.070), (0.070, -0.070), (-0.070, 0.070), (0.070, 0.070)),
            start=1,
        ):
            grate.visual(
                Box((0.018, 0.018, 0.012)),
                origin=Origin(xyz=(foot_x, foot_y, 0.006)),
                material=grate_iron,
                name=f"foot_{foot_index}",
            )
        grate.inertial = Inertial.from_geometry(
            Box((0.180, 0.180, 0.024)),
            mass=1.1,
            origin=Origin(xyz=(0.000, 0.000, 0.012)),
        )
        model.articulation(
            f"body_to_{burner_name}_grate",
            ArticulationType.FIXED,
            parent=body,
            child=grate,
            origin=Origin(xyz=(x_pos, y_pos, 0.860)),
        )

    for index, knob_x in enumerate((-0.240, -0.080, 0.080, 0.240), start=1):
        knob = model.part(f"control_knob_{index}")
        knob.visual(
            Cylinder(radius=0.010, length=0.004),
            origin=Origin(xyz=(0.000, 0.002, 0.000), rpy=(math.pi * 0.5, 0.0, 0.0)),
            material=dark_steel,
            name="collar",
        )
        knob.visual(
            Cylinder(radius=0.022, length=0.024),
            origin=Origin(xyz=(0.000, 0.012, 0.000), rpy=(math.pi * 0.5, 0.0, 0.0)),
            material=knob_metal,
            name="knob_body",
        )
        knob.visual(
            Box((0.008, 0.006, 0.010)),
            origin=Origin(xyz=(0.000, 0.023, 0.017)),
            material=dark_steel,
            name="indicator",
        )
        knob.inertial = Inertial.from_geometry(
            Cylinder(radius=0.022, length=0.028),
            mass=0.08,
            origin=Origin(xyz=(0.000, 0.014, 0.000)),
        )
        model.articulation(
            f"body_to_control_knob_{index}",
            ArticulationType.REVOLUTE,
            parent=body,
            child=knob,
            origin=Origin(xyz=(knob_x, -0.225, 0.905)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=1.0,
                velocity=3.0,
                lower=0.0,
                upper=math.radians(270.0),
            ),
        )

    door = model.part("oven_door")
    door.visual(
        Box((0.680, 0.024, 0.100)),
        origin=Origin(xyz=(0.000, 0.010, 0.050)),
        material=body_enamel,
        name="door_bottom_rail",
    )
    door.visual(
        Box((0.680, 0.024, 0.120)),
        origin=Origin(xyz=(0.000, 0.010, 0.520)),
        material=body_enamel,
        name="door_top_rail",
    )
    door.visual(
        Box((0.080, 0.024, 0.580)),
        origin=Origin(xyz=(-0.300, 0.010, 0.290)),
        material=body_enamel,
        name="door_left_rail",
    )
    door.visual(
        Box((0.080, 0.024, 0.580)),
        origin=Origin(xyz=(0.300, 0.010, 0.290)),
        material=body_enamel,
        name="door_right_rail",
    )
    door.visual(
        Box((0.520, 0.006, 0.360)),
        origin=Origin(xyz=(0.000, 0.015, 0.280)),
        material=glass,
        name="window_glass",
    )
    door.visual(
        Box((0.520, 0.008, 0.360)),
        origin=Origin(xyz=(0.000, 0.005, 0.280)),
        material=cavity_dark,
        name="window_shadow_panel",
    )
    door.visual(
        Cylinder(radius=0.010, length=0.660),
        origin=Origin(xyz=(0.000, 0.000, 0.000), rpy=(0.000, math.pi * 0.5, 0.000)),
        material=grate_iron,
        name="hinge_barrel",
    )
    door.visual(
        Box((0.030, 0.022, 0.050)),
        origin=Origin(xyz=(-0.245, 0.033, 0.540)),
        material=knob_metal,
        name="handle_left_post",
    )
    door.visual(
        Box((0.030, 0.022, 0.050)),
        origin=Origin(xyz=(0.245, 0.033, 0.540)),
        material=knob_metal,
        name="handle_right_post",
    )
    door.visual(
        Cylinder(radius=0.012, length=0.560),
        origin=Origin(xyz=(0.000, 0.046, 0.540), rpy=(0.000, math.pi * 0.5, 0.000)),
        material=knob_metal,
        name="handle_bar",
    )
    door.inertial = Inertial.from_geometry(
        Box((0.680, 0.060, 0.580)),
        mass=13.5,
        origin=Origin(xyz=(0.000, 0.020, 0.290)),
    )

    model.articulation(
        "oven_door_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(0.000, 0.298, 0.100)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=1.8, lower=-1.45, upper=0.0),
    )

    return model


def build_object_model() -> ArticulatedObject:
    return _build_gas_range()


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    door = object_model.get_part("oven_door")
    door_hinge = object_model.get_articulation("oven_door_hinge")

    upper_front_panel = body.get_visual("upper_front_panel")
    hinge_sill = body.get_visual("hinge_sill")
    reveal_top = body.get_visual("front_reveal_top")

    door_bottom_rail = door.get_visual("door_bottom_rail")
    door_top_rail = door.get_visual("door_top_rail")
    window_glass = door.get_visual("window_glass")
    shadow_panel = door.get_visual("window_shadow_panel")
    hinge_barrel = door.get_visual("hinge_barrel")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()
    ctx.fail_if_articulation_overlaps(max_pose_samples=64, overlap_tol=0.001, overlap_volume_tol=0.0)

    ctx.check(
        "door_hinge_axis_is_full_width_x",
        tuple(door_hinge.axis) == (1.0, 0.0, 0.0),
        "The oven door should hinge about the full-width bottom-edge x-axis.",
    )

    ctx.expect_overlap(door, body, axes="xz", min_overlap=0.25)
    ctx.expect_gap(
        door,
        body,
        axis="y",
        max_gap=0.012,
        max_penetration=0.0,
        positive_elem=door_top_rail,
        negative_elem=upper_front_panel,
    )
    ctx.expect_gap(
        door,
        body,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=door_bottom_rail,
        negative_elem=hinge_sill,
    )
    ctx.expect_contact(
        door,
        body,
        elem_a=hinge_barrel,
        elem_b=hinge_sill,
        contact_tol=0.001,
    )
    ctx.expect_within(door, door, axes="xz", inner_elem=window_glass)
    ctx.expect_within(door, door, axes="xz", inner_elem=shadow_panel)
    ctx.expect_gap(
        door,
        door,
        axis="y",
        min_gap=0.001,
        max_gap=0.012,
        positive_elem=window_glass,
        negative_elem=shadow_panel,
    )

    burner_names = ("front_left", "front_right", "rear_left", "rear_right")
    grate_parts = [object_model.get_part(f"{name}_grate") for name in burner_names]

    for index, burner_name in enumerate(burner_names, start=1):
        burner = object_model.get_part(f"{burner_name}_burner")
        grate = object_model.get_part(f"{burner_name}_grate")
        grate_center_bar = grate.get_visual("center_bar_x")
        knob = object_model.get_part(f"control_knob_{index}")
        knob_joint = object_model.get_articulation(f"body_to_control_knob_{index}")

        ctx.expect_contact(burner, body, name=f"{burner_name}_burner_contacts_cooktop")
        ctx.expect_contact(grate, body, name=f"{burner_name}_grate_is_supported")
        ctx.expect_within(grate, body, axes="xy", margin=0.0, name=f"{burner_name}_grate_within_range_plan")
        ctx.expect_within(burner, grate, axes="xy", margin=0.0, name=f"{burner_name}_burner_centered_under_grate")
        ctx.expect_gap(
            grate,
            burner,
            axis="z",
            min_gap=0.001,
            max_gap=0.010,
            positive_elem=grate_center_bar,
            name=f"{burner_name}_grate_clears_burner_cap",
        )
        ctx.expect_contact(knob, body, name=f"{knob.name}_mounted_to_panel")
        ctx.check(
            f"{knob.name}_joint_axis",
            tuple(knob_joint.axis) == (0.0, 1.0, 0.0),
            "Control knobs should rotate about the front-back axis.",
        )

    ctx.expect_gap(
        grate_parts[1],
        grate_parts[0],
        axis="x",
        min_gap=0.140,
        name="front_grates_have_side_clearance",
    )
    ctx.expect_gap(
        grate_parts[3],
        grate_parts[2],
        axis="x",
        min_gap=0.140,
        name="rear_grates_have_side_clearance",
    )
    ctx.expect_gap(
        grate_parts[0],
        grate_parts[2],
        axis="y",
        min_gap=0.050,
        name="left_grates_have_realistic_fore_aft_clearance",
    )
    ctx.expect_gap(
        grate_parts[1],
        grate_parts[3],
        axis="y",
        min_gap=0.050,
        name="right_grates_have_realistic_fore_aft_clearance",
    )

    door_limits = door_hinge.motion_limits
    assert door_limits is not None and door_limits.lower is not None and door_limits.upper is not None
    with ctx.pose({door_hinge: door_limits.lower}):
        ctx.fail_if_parts_overlap_in_current_pose(name="oven_door_open_no_overlap")
        ctx.fail_if_isolated_parts(name="oven_door_open_no_floating")
        ctx.expect_contact(
            door,
            body,
            elem_a=hinge_barrel,
            elem_b=hinge_sill,
            contact_tol=0.001,
            name="oven_door_open_hinge_contact",
        )
        ctx.expect_gap(
            body,
            door,
            axis="z",
            min_gap=0.250,
            positive_elem=reveal_top,
            negative_elem=door_top_rail,
            name="oven_door_opens_downward",
        )
        ctx.expect_gap(
            door,
            body,
            axis="y",
            min_gap=0.200,
            positive_elem=door_top_rail,
            negative_elem=upper_front_panel,
            name="oven_door_swings_forward",
        )

    def aabb_center(
        aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None,
    ) -> tuple[float, float, float] | None:
        if aabb is None:
            return None
        return tuple((lo + hi) * 0.5 for lo, hi in zip(aabb[0], aabb[1]))

    for index in range(1, 5):
        knob = object_model.get_part(f"control_knob_{index}")
        knob_joint = object_model.get_articulation(f"body_to_control_knob_{index}")
        knob_limits = knob_joint.motion_limits
        assert knob_limits is not None and knob_limits.upper is not None

        rest_center = aabb_center(ctx.part_element_world_aabb(knob, elem="indicator"))
        moved_center = None
        with ctx.pose({knob_joint: knob_limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name=f"{knob.name}_max_turn_no_overlap")
            ctx.fail_if_isolated_parts(name=f"{knob.name}_max_turn_no_floating")
            ctx.expect_contact(knob, body, name=f"{knob.name}_still_mounted_at_max_turn")
            moved_center = aabb_center(ctx.part_element_world_aabb(knob, elem="indicator"))

        indicator_ok = (
            rest_center is not None
            and moved_center is not None
            and abs(moved_center[1] - rest_center[1]) < 0.001
            and (
                abs(moved_center[0] - rest_center[0]) > 0.006
                or abs(moved_center[2] - rest_center[2]) > 0.006
            )
        )
        ctx.check(
            f"{knob.name}_indicator_rotates_with_joint",
            indicator_ok,
            "The knob indicator should move around the knob face when the joint turns.",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
