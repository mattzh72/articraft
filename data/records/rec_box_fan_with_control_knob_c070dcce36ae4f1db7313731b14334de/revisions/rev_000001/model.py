from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BezelFace,
    BezelGeometry,
    Box,
    Cylinder,
    ExtrudeWithHolesGeometry,
    FanRotorBlade,
    FanRotorGeometry,
    FanRotorHub,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _rect_profile(width: float, height: float) -> list[tuple[float, float]]:
    half_w = width / 2.0
    half_h = height / 2.0
    return [
        (-half_w, -half_h),
        (half_w, -half_h),
        (half_w, half_h),
        (-half_w, half_h),
    ]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="industrial_wall_exhaust_fan")

    galvanized = model.material("galvanized_steel", rgba=(0.58, 0.62, 0.64, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.08, 0.09, 0.10, 1.0))
    wall_mat = model.material("painted_wall", rgba=(0.70, 0.72, 0.70, 1.0))
    blade_mat = model.material("dark_blades", rgba=(0.12, 0.14, 0.15, 1.0))
    knob_mat = model.material("black_knob", rgba=(0.02, 0.02, 0.018, 1.0))
    warning_label = model.material("yellow_label", rgba=(0.95, 0.72, 0.10, 1.0))

    housing = model.part("wall_housing")

    wall_panel = ExtrudeWithHolesGeometry(
        _rect_profile(1.05, 0.95),
        [rounded_rect_profile(0.54, 0.54, 0.010, corner_segments=4)],
        0.040,
        center=True,
    )
    housing.visual(
        mesh_from_geometry(wall_panel, "wall_opening_panel"),
        origin=Origin(rpy=(-pi / 2.0, 0.0, 0.0)),
        material=wall_mat,
        name="wall_opening",
    )

    outlet_frame = BezelGeometry(
        opening_size=(0.54, 0.54),
        outer_size=(0.72, 0.72),
        depth=0.160,
        opening_shape="rounded_rect",
        outer_shape="rounded_rect",
        opening_corner_radius=0.012,
        outer_corner_radius=0.020,
        face=BezelFace(style="radiused_step", front_lip=0.010, fillet=0.004),
        center=False,
    )
    housing.visual(
        mesh_from_geometry(outlet_frame, "square_outlet_frame"),
        origin=Origin(xyz=(0.0, 0.018, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=galvanized,
        name="outlet_frame",
    )

    # A deeper square duct body behind the bolted face frame.  The four panels
    # leave the middle open, matching an exhaust fan shroud rather than a solid box.
    housing.visual(
        Box((0.65, 0.155, 0.055)),
        origin=Origin(xyz=(0.0, 0.095, 0.2975)),
        material=galvanized,
        name="top_duct_wall",
    )
    housing.visual(
        Box((0.65, 0.155, 0.055)),
        origin=Origin(xyz=(0.0, 0.095, -0.2975)),
        material=galvanized,
        name="bottom_duct_wall",
    )
    housing.visual(
        Box((0.055, 0.155, 0.65)),
        origin=Origin(xyz=(0.2975, 0.095, 0.0)),
        material=galvanized,
        name="side_duct_wall_0",
    )
    housing.visual(
        Box((0.055, 0.155, 0.65)),
        origin=Origin(xyz=(-0.2975, 0.095, 0.0)),
        material=galvanized,
        name="side_duct_wall_1",
    )

    # Four visible bolt heads clamp the square housing to the wall.
    for index, (x, z) in enumerate(
        ((0.315, 0.315), (-0.315, 0.315), (-0.315, -0.315), (0.315, -0.315))
    ):
        housing.visual(
            Cylinder(radius=0.018, length=0.014),
            origin=Origin(xyz=(x, 0.183, z), rpy=(-pi / 2.0, 0.0, 0.0)),
            material=dark_steel,
            name=f"bolt_head_{index}",
        )

    # Stationary motor can and simple cross braces inside the housing.
    housing.visual(
        Cylinder(radius=0.070, length=0.042),
        origin=Origin(xyz=(0.0, 0.039, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="motor_can",
    )
    housing.visual(
        Box((0.018, 0.020, 0.46)),
        origin=Origin(xyz=(0.0, 0.045, 0.0), rpy=(0.0, 0.0, pi / 4.0)),
        material=dark_steel,
        name="motor_strut_0",
    )
    housing.visual(
        Box((0.018, 0.020, 0.46)),
        origin=Origin(xyz=(0.0, 0.045, 0.0), rpy=(0.0, 0.0, -pi / 4.0)),
        material=dark_steel,
        name="motor_strut_1",
    )

    # Side control boss: the knob part mounts to its flat outer face.
    housing.visual(
        Cylinder(radius=0.040, length=0.022),
        origin=Origin(xyz=(0.363, 0.105, -0.165), rpy=(0.0, pi / 2.0, 0.0)),
        material=galvanized,
        name="control_boss",
    )
    housing.visual(
        Box((0.006, 0.080, 0.032)),
        origin=Origin(xyz=(0.374, 0.105, -0.215)),
        material=warning_label,
        name="control_label",
    )

    # Two hinge cheeks support the exterior gravity damper at the upper edge.
    for index, x in enumerate((-0.335, 0.335)):
        housing.visual(
            Box((0.040, 0.038, 0.060)),
            origin=Origin(xyz=(x, 0.191, 0.310)),
            material=galvanized,
            name=f"hinge_cheek_{index}",
        )

    fan_rotor = model.part("fan_rotor")
    rotor_mesh = FanRotorGeometry(
        outer_radius=0.225,
        hub_radius=0.058,
        blade_count=6,
        thickness=0.045,
        blade_pitch_deg=32.0,
        blade_sweep_deg=28.0,
        blade=FanRotorBlade(shape="scimitar", tip_pitch_deg=16.0, camber=0.10, tip_clearance=0.004),
        hub=FanRotorHub(style="spinner", rear_collar_height=0.015, rear_collar_radius=0.042),
    )
    fan_rotor.visual(
        mesh_from_geometry(rotor_mesh, "six_blade_rotor"),
        material=blade_mat,
        name="rotor",
    )
    fan_rotor.visual(
        Cylinder(radius=0.016, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, -0.020)),
        material=dark_steel,
        name="shaft_stub",
    )

    damper_flap = model.part("damper_flap")
    damper_flap.visual(
        Box((0.600, 0.014, 0.560)),
        origin=Origin(xyz=(0.0, 0.007, -0.280)),
        material=galvanized,
        name="flap_panel",
    )
    damper_flap.visual(
        Cylinder(radius=0.011, length=0.630),
        origin=Origin(xyz=(0.0, 0.007, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_steel,
        name="flap_hinge_barrel",
    )
    damper_flap.visual(
        Box((0.560, 0.006, 0.030)),
        origin=Origin(xyz=(0.0, 0.016, -0.045)),
        material=dark_steel,
        name="stiffening_rib",
    )

    control_knob = model.part("control_knob")
    knob_mesh = KnobGeometry(
        diameter=0.070,
        height=0.045,
        body_style="faceted",
        top_diameter=0.055,
        edge_radius=0.0015,
        grip=KnobGrip(style="ribbed", count=18, depth=0.0015, width=0.003),
        indicator=KnobIndicator(style="line", mode="raised", angle_deg=0.0),
        center=False,
    )
    control_knob.visual(
        mesh_from_geometry(knob_mesh, "side_control_knob"),
        material=knob_mat,
        name="knob_cap",
    )

    model.articulation(
        "housing_to_fan",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=fan_rotor,
        origin=Origin(xyz=(0.0, 0.105, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=45.0),
    )
    model.articulation(
        "housing_to_flap",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=damper_flap,
        origin=Origin(xyz=(0.0, 0.194, 0.310)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.6, lower=0.0, upper=1.25),
    )
    model.articulation(
        "housing_to_knob",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=control_knob,
        origin=Origin(xyz=(0.374, 0.105, -0.165), rpy=(0.0, pi / 2.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.0, velocity=8.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    housing = object_model.get_part("wall_housing")
    fan = object_model.get_part("fan_rotor")
    flap = object_model.get_part("damper_flap")
    knob = object_model.get_part("control_knob")
    flap_joint = object_model.get_articulation("housing_to_flap")

    ctx.expect_within(
        fan,
        housing,
        axes="xz",
        inner_elem="rotor",
        outer_elem="outlet_frame",
        margin=0.004,
        name="fan rotor sits inside square housing opening",
    )
    ctx.expect_gap(
        flap,
        housing,
        axis="y",
        positive_elem="flap_panel",
        negative_elem="outlet_frame",
        min_gap=0.005,
        max_gap=0.030,
        name="closed damper flap is just outside outlet face",
    )
    ctx.expect_gap(
        knob,
        housing,
        axis="x",
        positive_elem="knob_cap",
        negative_elem="control_boss",
        max_penetration=0.001,
        max_gap=0.004,
        name="side control knob seats on boss",
    )

    closed_aabb = ctx.part_element_world_aabb(flap, elem="flap_panel")
    with ctx.pose({flap_joint: 1.0}):
        open_aabb = ctx.part_element_world_aabb(flap, elem="flap_panel")
    ctx.check(
        "damper flap opens outward from top hinge",
        closed_aabb is not None
        and open_aabb is not None
        and open_aabb[1][1] > closed_aabb[1][1] + 0.25
        and open_aabb[0][2] > closed_aabb[0][2] - 0.03,
        details=f"closed={closed_aabb}, open={open_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
