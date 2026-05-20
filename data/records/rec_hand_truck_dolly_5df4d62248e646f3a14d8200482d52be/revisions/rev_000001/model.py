from __future__ import annotations

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
    TireGeometry,
    TireShoulder,
    TireSidewall,
    TireTread,
    WheelBore,
    WheelFace,
    WheelGeometry,
    WheelHub,
    WheelRim,
    WheelSpokes,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _tube_mesh(points, radius: float, name: str, *, samples: int = 10):
    return mesh_from_geometry(
        tube_from_spline_points(
            points,
            radius=radius,
            samples_per_segment=samples,
            radial_segments=18,
            cap_ends=True,
        ),
        name,
    )


def _add_hand_truck_wheel(part, prefix: str, *, rubber, zinc, dark):
    """Large utility wheel: block-tread tire, open rim, and raised hub."""
    wheel_origin = Origin(rpy=(0.0, 0.0, pi / 2.0))
    tire = mesh_from_geometry(
        TireGeometry(
            0.185,
            0.080,
            inner_radius=0.126,
            tread=TireTread(style="block", depth=0.010, count=18, land_ratio=0.54),
            sidewall=TireSidewall(style="square", bulge=0.025),
            shoulder=TireShoulder(width=0.010, radius=0.004),
        ),
        f"{prefix}_tire",
    )
    rim = mesh_from_geometry(
        WheelGeometry(
            0.126,
            0.060,
            rim=WheelRim(
                inner_radius=0.070,
                flange_height=0.009,
                flange_thickness=0.004,
                bead_seat_depth=0.004,
            ),
            hub=WheelHub(radius=0.040, width=0.070, cap_style="flat"),
            face=WheelFace(dish_depth=0.006, front_inset=0.003, rear_inset=0.003),
            spokes=WheelSpokes(style="split_y", count=5, thickness=0.004, window_radius=0.014),
            bore=WheelBore(style="round", diameter=0.036),
        ),
        f"{prefix}_rim",
    )
    part.visual(tire, origin=wheel_origin, material=rubber, name="tire")
    part.visual(rim, origin=wheel_origin, material=zinc, name="rim")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="convertible_hand_truck")

    red = model.material("powder_coated_red", rgba=(0.73, 0.05, 0.03, 1.0))
    steel = model.material("dark_steel", rgba=(0.18, 0.19, 0.18, 1.0))
    plate = model.material("scratched_plate", rgba=(0.55, 0.56, 0.53, 1.0))
    zinc = model.material("zinc_rim", rgba=(0.72, 0.72, 0.68, 1.0))
    rubber = model.material("black_rubber", rgba=(0.035, 0.035, 0.035, 1.0))

    frame = model.part("frame")
    # Primary load toe plate and vertical back lip.
    frame.visual(
        Box((0.420, 0.500, 0.025)),
        origin=Origin(xyz=(0.310, 0.0, 0.055)),
        material=plate,
        name="primary_plate",
    )
    frame.visual(
        Box((0.045, 0.500, 0.160)),
        origin=Origin(xyz=(0.080, 0.0, 0.125)),
        material=plate,
        name="toe_back_lip",
    )
    frame.visual(
        Box((0.060, 0.440, 0.030)),
        origin=Origin(xyz=(0.095, 0.0, 0.080)),
        material=steel,
        name="plate_root_bar",
    )

    # Bent tubular uprights and handle.
    for side, y in (("side_0", -0.205), ("side_1", 0.205)):
        rail_points = [
            (0.070, y, 0.105),
            (0.025, y, 0.340),
            (-0.050, y, 0.760),
            (-0.150, y, 1.230),
            (-0.112, y, 1.385),
        ]
        frame.visual(
            _tube_mesh(rail_points, 0.022, f"{side}_upright", samples=14),
            material=red,
            name=f"{side}_upright",
        )
        frame.visual(
            Box((0.070, 0.054, 0.075)),
            origin=Origin(xyz=(0.035, y, 0.185)),
            material=steel,
            name=f"{side}_axle_bracket",
        )

    frame.visual(
        _tube_mesh(
            [
                (-0.112, -0.205, 1.385),
                (-0.145, -0.090, 1.430),
                (-0.145, 0.090, 1.430),
                (-0.112, 0.205, 1.385),
            ],
            0.024,
            "top_handle",
            samples=14,
        ),
        material=red,
        name="top_handle",
    )
    for z, x, name, radius in (
        (0.430, 0.005, "lower_crossbar", 0.018),
        (0.840, -0.065, "middle_crossbar", 0.017),
        (1.170, -0.135, "upper_crossbar", 0.017),
    ):
        frame.visual(
            Cylinder(radius=radius, length=0.455),
            origin=Origin(xyz=(x, 0.0, z), rpy=(pi / 2.0, 0.0, 0.0)),
            material=red,
            name=name,
        )
    frame.visual(
        Cylinder(radius=0.018, length=0.740),
        origin=Origin(xyz=(-0.030, 0.0, 0.205), rpy=(pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="axle",
    )
    frame.visual(
        _tube_mesh(
            [(0.060, -0.210, 0.140), (-0.005, -0.235, 0.185), (-0.055, -0.260, 0.205)],
            0.016,
            "wheel_guard_0",
            samples=8,
        ),
        material=steel,
        name="wheel_guard_0",
    )
    frame.visual(
        _tube_mesh(
            [(0.060, 0.210, 0.140), (-0.005, 0.235, 0.185), (-0.055, 0.260, 0.205)],
            0.016,
            "wheel_guard_1",
            samples=8,
        ),
        material=steel,
        name="wheel_guard_1",
    )

    # Interleaved hinge knuckles under the front edge of the toe plate.
    for y, name in ((-0.170, "hinge_knuckle_0"), (0.170, "hinge_knuckle_1")):
        frame.visual(
            Cylinder(radius=0.012, length=0.110),
            origin=Origin(xyz=(0.520, y, 0.035), rpy=(pi / 2.0, 0.0, 0.0)),
            material=steel,
            name=name,
        )
        frame.visual(
            Box((0.050, 0.105, 0.006)),
            origin=Origin(xyz=(0.500, y, 0.041)),
            material=steel,
            name=f"{name}_leaf",
        )

    wheel_0 = model.part("wheel_0")
    _add_hand_truck_wheel(wheel_0, "wheel_0", rubber=rubber, zinc=zinc, dark=steel)
    wheel_1 = model.part("wheel_1")
    _add_hand_truck_wheel(wheel_1, "wheel_1", rubber=rubber, zinc=zinc, dark=steel)

    extension = model.part("extension_plate")
    extension.visual(
        Box((0.420, 0.430, 0.016)),
        origin=Origin(xyz=(-0.210, 0.0, -0.022)),
        material=plate,
        name="extension_plate",
    )
    extension.visual(
        Box((0.390, 0.018, 0.022)),
        origin=Origin(xyz=(-0.225, -0.180, -0.010)),
        material=steel,
        name="side_rib_0",
    )
    extension.visual(
        Box((0.390, 0.018, 0.022)),
        origin=Origin(xyz=(-0.225, 0.180, -0.010)),
        material=steel,
        name="side_rib_1",
    )
    extension.visual(
        Box((0.024, 0.360, 0.018)),
        origin=Origin(xyz=(-0.410, 0.0, -0.010)),
        material=steel,
        name="front_lip",
    )
    extension.visual(
        Box((0.080, 0.205, 0.006)),
        origin=Origin(xyz=(-0.040, 0.0, -0.014)),
        material=steel,
        name="hinge_leaf",
    )
    extension.visual(
        Cylinder(radius=0.011, length=0.205),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="hinge_barrel",
    )

    model.articulation(
        "frame_to_wheel_0",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=wheel_0,
        origin=Origin(xyz=(-0.030, -0.330, 0.205)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=25.0),
    )
    model.articulation(
        "frame_to_wheel_1",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=wheel_1,
        origin=Origin(xyz=(-0.030, 0.330, 0.205)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=25.0),
    )
    model.articulation(
        "toe_extension_fold",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=extension,
        origin=Origin(xyz=(0.520, 0.0, 0.035)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=2.0, lower=0.0, upper=pi),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    extension = object_model.get_part("extension_plate")
    wheel_0 = object_model.get_part("wheel_0")
    wheel_1 = object_model.get_part("wheel_1")
    fold = object_model.get_articulation("toe_extension_fold")
    spin_0 = object_model.get_articulation("frame_to_wheel_0")
    spin_1 = object_model.get_articulation("frame_to_wheel_1")

    ctx.allow_overlap(
        frame,
        wheel_0,
        elem_a="axle",
        elem_b="rim",
        reason="The axle is intentionally captured through the simplified wheel hub and bearing mesh.",
    )
    ctx.allow_overlap(
        frame,
        wheel_1,
        elem_a="axle",
        elem_b="rim",
        reason="The axle is intentionally captured through the simplified wheel hub and bearing mesh.",
    )

    ctx.check(
        "wheels use continuous spin joints",
        str(spin_0.articulation_type).endswith("CONTINUOUS")
        and str(spin_1.articulation_type).endswith("CONTINUOUS"),
    )
    ctx.check(
        "fold-out toe section has about 180 degrees of travel",
        fold.motion_limits is not None
        and fold.motion_limits.lower == 0.0
        and abs(fold.motion_limits.upper - pi) < 1e-6,
    )

    with ctx.pose({fold: 0.0}):
        ctx.expect_within(
            extension,
            frame,
            axes="xy",
            inner_elem="extension_plate",
            outer_elem="primary_plate",
            margin=0.004,
            name="extension plate nests under the primary nose plate",
        )
        ctx.expect_gap(
            frame,
            extension,
            axis="z",
            positive_elem="primary_plate",
            negative_elem="extension_plate",
            min_gap=0.014,
            max_gap=0.024,
            name="folded extension is tucked below the main plate",
        )

    with ctx.pose({fold: pi}):
        ctx.expect_gap(
            extension,
            frame,
            axis="x",
            positive_elem="extension_plate",
            negative_elem="primary_plate",
            max_gap=0.004,
            max_penetration=0.004,
            name="deployed extension starts at the front edge of the nose plate",
        )
        ctx.expect_overlap(
            extension,
            frame,
            axes="y",
            elem_a="extension_plate",
            elem_b="primary_plate",
            min_overlap=0.38,
            name="deployed extension stays width-aligned with the main plate",
        )

    ctx.expect_origin_gap(
        wheel_1,
        wheel_0,
        axis="y",
        min_gap=0.60,
        max_gap=0.70,
        name="two rear wheels sit on opposite axle ends",
    )
    for wheel, name in ((wheel_0, "wheel_0"), (wheel_1, "wheel_1")):
        ctx.expect_within(
            frame,
            wheel,
            axes="xz",
            inner_elem="axle",
            outer_elem="rim",
            margin=0.0,
            name=f"{name} hub surrounds the axle",
        )
        ctx.expect_overlap(
            frame,
            wheel,
            axes="y",
            elem_a="axle",
            elem_b="rim",
            min_overlap=0.055,
            name=f"{name} rim remains axially captured on axle",
        )

    return ctx.report()


object_model = build_object_model()
