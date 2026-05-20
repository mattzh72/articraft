from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    Cylinder,
    CylinderGeometry,
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    Inertial,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pyramid_metronome")

    wood = model.material("wood", rgba=(0.34, 0.20, 0.10, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.10, 0.10, 0.11, 1.0))
    brass = model.material("brass", rgba=(0.79, 0.64, 0.27, 1.0))
    steel = model.material("steel", rgba=(0.70, 0.72, 0.75, 1.0))

    base_w = 0.130
    base_d = 0.146
    top_w = 0.042
    top_d = 0.052
    body_h = 0.232
    base_t = 0.012
    panel_t = 0.006
    panel_z0 = 0.004
    pivot_y = base_d / 2.0 + 0.010
    pivot_z = panel_z0 + body_h + 0.010

    def trapezoid_profile(bottom_width: float, top_width: float, height: float) -> list[tuple[float, float]]:
        return [
            (-bottom_width / 2.0, 0.0),
            (bottom_width / 2.0, 0.0),
            (top_width / 2.0, height),
            (-top_width / 2.0, height),
        ]

    def build_front_panel() -> MeshGeometry:
        front_outer = trapezoid_profile(base_w - 0.004, top_w + 0.008, body_h)
        front_slot = [
            (-0.007, 0.020),
            (0.007, 0.020),
            (0.007, 0.176),
            (-0.007, 0.176),
        ]
        front_panel = ExtrudeWithHolesGeometry(front_outer, [front_slot], panel_t, center=True)
        front_panel.rotate_x(math.pi / 2.0).translate(0.0, base_d / 2.0 - panel_t / 2.0, panel_z0)
        return front_panel

    def build_back_panel() -> MeshGeometry:
        back_outer = trapezoid_profile(base_w - 0.012, top_w + 0.012, body_h)
        back_panel = ExtrudeGeometry(back_outer, panel_t, center=True)
        back_panel.rotate_x(math.pi / 2.0).translate(0.0, -base_d / 2.0 + panel_t / 2.0, panel_z0)
        return back_panel

    def build_side_panel(x_sign: float) -> MeshGeometry:
        side_profile = trapezoid_profile(base_d - 0.010, top_d + 0.008, body_h)
        side = ExtrudeGeometry(side_profile, panel_t, center=True)
        side.rotate_x(math.pi / 2.0).rotate_z(math.pi / 2.0)
        side.translate(x_sign * (base_w / 2.0 - panel_t / 2.0), 0.0, panel_z0)
        return side

    housing = model.part("housing")
    housing.visual(
        Box((base_w, base_d, base_t)),
        origin=Origin(xyz=(0.0, 0.0, base_t / 2.0)),
        material=wood,
        name="base_plate",
    )
    housing.visual(
        mesh_from_geometry(build_front_panel(), "front_panel"),
        material=wood,
        name="front_panel",
    )
    housing.visual(
        mesh_from_geometry(build_back_panel(), "back_panel"),
        material=wood,
        name="back_panel",
    )
    housing.visual(
        mesh_from_geometry(build_side_panel(1.0), "left_side_panel"),
        material=wood,
        name="left_side_panel",
    )
    housing.visual(
        mesh_from_geometry(build_side_panel(-1.0), "right_side_panel"),
        material=wood,
        name="right_side_panel",
    )
    housing.visual(
        Box((top_w + 0.022, top_d + 0.026, 0.024)),
        origin=Origin(xyz=(0.0, 0.0, panel_z0 + body_h + 0.002)),
        material=wood,
        name="top_cap",
    )
    housing.visual(
        Box((0.024, 0.050, 0.010)),
        origin=Origin(xyz=(0.0, 0.055, panel_z0 + body_h + 0.008)),
        material=wood,
        name="top_support_bridge",
    )
    housing.visual(
        Box((0.016, 0.022, 0.044)),
        origin=Origin(xyz=(0.0, base_d / 2.0 - 0.011, panel_z0 + body_h - 0.010)),
        material=wood,
        name="front_crown_support",
    )
    housing.visual(
        Box((0.022, 0.016, 0.020)),
        origin=Origin(xyz=(base_w / 2.0 - 0.011, -base_d / 2.0 + 0.008, 0.010)),
        material=dark_trim,
        name="left_rear_hinge_block",
    )
    housing.visual(
        Box((0.022, 0.016, 0.020)),
        origin=Origin(xyz=(-base_w / 2.0 + 0.011, -base_d / 2.0 + 0.008, 0.010)),
        material=dark_trim,
        name="right_rear_hinge_block",
    )
    housing.visual(
        Box((0.020, 0.014, 0.006)),
        origin=Origin(xyz=(base_w / 2.0 - 0.018, base_d / 2.0 - 0.012, 0.003)),
        material=dark_trim,
        name="left_front_foot",
    )
    housing.visual(
        Box((0.020, 0.014, 0.006)),
        origin=Origin(xyz=(-base_w / 2.0 + 0.018, base_d / 2.0 - 0.012, 0.003)),
        material=dark_trim,
        name="right_front_foot",
    )
    housing.visual(
        Cylinder(radius=0.0055, length=0.045),
        origin=Origin(
            xyz=(0.0, pivot_y - 0.025, pivot_z),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=brass,
        name="top_pivot_boss",
    )
    housing.inertial = Inertial.from_geometry(
        Box((base_w, base_d, panel_z0 + body_h + 0.022)),
        mass=1.8,
        origin=Origin(xyz=(0.0, 0.0, (panel_z0 + body_h + 0.022) / 2.0)),
    )

    pendulum = model.part("pendulum")
    pendulum.visual(
        Cylinder(radius=0.0025, length=0.172),
        origin=Origin(xyz=(0.0, 0.0, 0.086)),
        material=steel,
        name="upper_rod",
    )
    pendulum.visual(
        Cylinder(radius=0.0025, length=0.222),
        origin=Origin(xyz=(0.0, 0.0, -0.111)),
        material=steel,
        name="lower_rod",
    )
    pendulum.visual(
        Cylinder(radius=0.018, length=0.012),
        origin=Origin(
            xyz=(0.0, 0.0, -0.184),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=brass,
        name="lower_bob",
    )
    pendulum.inertial = Inertial.from_geometry(
        Box((0.040, 0.020, 0.410)),
        mass=0.22,
        origin=Origin(xyz=(0.0, 0.0, -0.020)),
    )

    slider_weight = model.part("slider_weight")
    slider_weight.visual(
        Cylinder(radius=0.014, length=0.018),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brass,
        name="slider_body",
    )
    slider_weight.inertial = Inertial.from_geometry(
        Cylinder(radius=0.014, length=0.018),
        mass=0.08,
        origin=Origin(),
    )

    def add_leg(name: str) -> None:
        leg = model.part(name)
        leg.visual(
            Box((0.018, 0.004, 0.120)),
            origin=Origin(xyz=(0.0, -0.002, 0.060)),
            material=dark_trim,
            name="leg_blade",
        )
        leg.visual(
            Box((0.028, 0.006, 0.020)),
            origin=Origin(xyz=(0.0, -0.0025, 0.118)),
            material=dark_trim,
            name="foot_pad",
        )
        leg.visual(
            Cylinder(radius=0.0055, length=0.020),
            origin=Origin(
                xyz=(0.0, -0.002, 0.0),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=dark_trim,
            name="hinge_knuckle",
        )
        leg.inertial = Inertial.from_geometry(
            Box((0.028, 0.012, 0.132)),
            mass=0.05,
            origin=Origin(xyz=(0.0, -0.002, 0.066)),
        )

    add_leg("left_leg")
    add_leg("right_leg")

    winding_key = model.part("winding_key")
    winding_key.visual(
        Cylinder(radius=0.004, length=0.016),
        origin=Origin(
            xyz=(0.0, -0.006, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=brass,
        name="key_stem",
    )
    winding_key.visual(
        Cylinder(radius=0.008, length=0.012),
        origin=Origin(
            xyz=(0.0, -0.018, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=brass,
        name="key_hub",
    )
    winding_key.visual(
        Box((0.046, 0.008, 0.010)),
        origin=Origin(xyz=(0.0, -0.026, 0.0)),
        material=brass,
        name="key_wing",
    )
    winding_key.visual(
        Box((0.014, 0.008, 0.026)),
        origin=Origin(xyz=(0.0, -0.026, 0.0)),
        material=brass,
        name="key_cross",
    )
    winding_key.inertial = Inertial.from_geometry(
        Box((0.046, 0.032, 0.026)),
        mass=0.04,
        origin=Origin(xyz=(0.0, -0.022, 0.0)),
    )

    model.articulation(
        "housing_to_pendulum",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=pendulum,
        origin=Origin(xyz=(0.0, pivot_y, pivot_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.8,
            velocity=2.5,
            lower=-0.42,
            upper=0.42,
        ),
    )

    model.articulation(
        "pendulum_to_slider_weight",
        ArticulationType.PRISMATIC,
        parent=pendulum,
        child=slider_weight,
        origin=Origin(xyz=(0.0, 0.0, 0.078)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=0.3,
            velocity=0.10,
            lower=0.0,
            upper=0.080,
        ),
    )

    model.articulation(
        "housing_to_left_leg",
        ArticulationType.REVOLUTE,
        parent=housing,
        child="left_leg",
        origin=Origin(xyz=(base_w / 2.0 - 0.010, -base_d / 2.0, 0.004)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=2.0,
            lower=0.0,
            upper=1.52,
        ),
    )

    model.articulation(
        "housing_to_right_leg",
        ArticulationType.REVOLUTE,
        parent=housing,
        child="right_leg",
        origin=Origin(xyz=(-base_w / 2.0 + 0.010, -base_d / 2.0, 0.004)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=2.0,
            lower=0.0,
            upper=1.52,
        ),
    )

    model.articulation(
        "housing_to_winding_key",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=winding_key,
        origin=Origin(xyz=(0.0, -base_d / 2.0 - 0.002, 0.072)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=0.4, velocity=12.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.

    housing = object_model.get_part("housing")
    pendulum = object_model.get_part("pendulum")
    slider_weight = object_model.get_part("slider_weight")
    left_leg = object_model.get_part("left_leg")
    right_leg = object_model.get_part("right_leg")
    winding_key = object_model.get_part("winding_key")

    pendulum_joint = object_model.get_articulation("housing_to_pendulum")
    slider_joint = object_model.get_articulation("pendulum_to_slider_weight")
    left_leg_joint = object_model.get_articulation("housing_to_left_leg")
    right_leg_joint = object_model.get_articulation("housing_to_right_leg")

    ctx.allow_overlap(
        pendulum,
        slider_weight,
        elem_a="upper_rod",
        elem_b="slider_body",
        reason="The cylindrical tempo weight intentionally rides around the pendulum rod.",
    )

    ctx.expect_gap(
        slider_weight,
        housing,
        axis="z",
        min_gap=0.058,
        negative_elem="top_support_bridge",
        name="slider weight sits above the housing apex",
    )
    ctx.expect_gap(
        pendulum,
        housing,
        axis="y",
        min_gap=0.0,
        positive_elem="lower_rod",
        negative_elem="front_panel",
        name="pendulum rod clears the front housing shell",
    )

    rest_slider_pos = ctx.part_world_position(slider_weight)
    with ctx.pose({slider_joint: slider_joint.motion_limits.upper}):
        high_slider_pos = ctx.part_world_position(slider_weight)
    ctx.check(
        "slider weight moves upward along the rod",
        rest_slider_pos is not None
        and high_slider_pos is not None
        and high_slider_pos[2] > rest_slider_pos[2] + 0.070,
        details=f"rest={rest_slider_pos}, high={high_slider_pos}",
    )

    rest_pendulum_aabb = ctx.part_world_aabb(pendulum)
    with ctx.pose({pendulum_joint: pendulum_joint.motion_limits.upper}):
        swung_pendulum_aabb = ctx.part_world_aabb(pendulum)
    ctx.check(
        "pendulum swings laterally from center",
        rest_pendulum_aabb is not None
        and swung_pendulum_aabb is not None
        and swung_pendulum_aabb[1][0] > rest_pendulum_aabb[1][0] + 0.045,
        details=f"rest={rest_pendulum_aabb}, swung={swung_pendulum_aabb}",
    )

    housing_aabb = ctx.part_world_aabb(housing)
    key_aabb = ctx.part_world_aabb(winding_key)
    ctx.check(
        "winding key remains on the rear face",
        housing_aabb is not None
        and key_aabb is not None
        and key_aabb[1][1] <= housing_aabb[0][1] + 0.004,
        details=f"housing={housing_aabb}, key={key_aabb}",
    )

    rest_left_aabb = ctx.part_world_aabb(left_leg)
    rest_right_aabb = ctx.part_world_aabb(right_leg)
    with ctx.pose(
        {
            left_leg_joint: left_leg_joint.motion_limits.upper,
            right_leg_joint: right_leg_joint.motion_limits.upper,
        }
    ):
        open_left_aabb = ctx.part_world_aabb(left_leg)
        open_right_aabb = ctx.part_world_aabb(right_leg)
    ctx.check(
        "left stabilizer leg folds backward for support",
        rest_left_aabb is not None
        and open_left_aabb is not None
        and open_left_aabb[0][1] < rest_left_aabb[0][1] - 0.090
        and open_left_aabb[0][2] < 0.020,
        details=f"rest={rest_left_aabb}, open={open_left_aabb}",
    )
    ctx.check(
        "right stabilizer leg folds backward for support",
        rest_right_aabb is not None
        and open_right_aabb is not None
        and open_right_aabb[0][1] < rest_right_aabb[0][1] - 0.090
        and open_right_aabb[0][2] < 0.020,
        details=f"rest={rest_right_aabb}, open={open_right_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
