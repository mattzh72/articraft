from __future__ import annotations

from math import atan2, pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BoltPattern,
    Box,
    Cylinder,
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
    TireCarcass,
    TireGeometry,
    TireGroove,
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
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="folding_platform_hand_truck")

    steel = model.material("powder_coated_steel", color=(0.08, 0.10, 0.12, 1.0))
    blue = model.material("blue_load_plate", color=(0.05, 0.16, 0.36, 1.0))
    black = model.material("black_rubber", color=(0.01, 0.01, 0.01, 1.0))
    dark = model.material("dark_polymer", color=(0.025, 0.025, 0.028, 1.0))
    zinc = model.material("zinc_plated_steel", color=(0.62, 0.64, 0.62, 1.0))
    shelf_paint = model.material("folding_shelf_blue", color=(0.04, 0.18, 0.42, 1.0))

    frame = model.part("frame")

    # Upright rectangular tube frame, about the size of a real hand truck.
    for y in (-0.22, 0.22):
        frame.visual(
            Box((0.035, 0.035, 1.12)),
            origin=Origin(xyz=(0.0, y, 0.72)),
            material=steel,
            name=f"upright_{0 if y < 0 else 1}",
        )

    for z, sx, sy, name in (
        (0.160, 0.045, 0.510, "bottom_rail"),
        (0.520, 0.035, 0.470, "middle_rail"),
        (0.920, 0.035, 0.470, "upper_rail"),
        (1.280, 0.042, 0.555, "handle_bar"),
    ):
        frame.visual(
            Box((sx, sy, 0.038)),
            origin=Origin(xyz=(0.0, 0.0, z)),
            material=steel,
            name=name,
        )

    for y in (-0.275, 0.275):
        frame.visual(
            Cylinder(radius=0.024, length=0.115),
            origin=Origin(xyz=(0.0, y, 1.280), rpy=(pi / 2.0, 0.0, 0.0)),
            material=dark,
            name=f"rubber_grip_{0 if y < 0 else 1}",
        )

    # Fixed primary toe plate and lower nose structure.
    frame.visual(
        Box((0.380, 0.500, 0.018)),
        origin=Origin(xyz=(0.190, 0.0, 0.035)),
        material=blue,
        name="primary_toe_plate",
    )
    frame.visual(
        Box((0.035, 0.520, 0.120)),
        origin=Origin(xyz=(0.015, 0.0, 0.094)),
        material=blue,
        name="toe_back_lip",
    )
    frame.visual(
        Cylinder(radius=0.012, length=0.500),
        origin=Origin(xyz=(0.382, 0.0, 0.047), rpy=(pi / 2.0, 0.0, 0.0)),
        material=blue,
        name="toe_front_roll",
    )
    for y in (-0.145, 0.0, 0.145):
        frame.visual(
            Box((0.285, 0.014, 0.006)),
            origin=Origin(xyz=(0.225, y, 0.0465)),
            material=zinc,
            name=f"toe_rib_{y:+.3f}",
        )

    # Side braces tie the toe plate into the upright tubes.
    brace_length = ((0.280**2) + (0.115**2)) ** 0.5
    brace_pitch = atan2(0.280, -0.115)
    for y in (-0.22, 0.22):
        frame.visual(
            Cylinder(radius=0.011, length=brace_length),
            origin=Origin(xyz=(0.140, y, 0.1025), rpy=(0.0, brace_pitch, 0.0)),
            material=steel,
            name=f"toe_brace_{0 if y < 0 else 1}",
        )

    # Axle and hinge support hardware.
    frame.visual(
        Box((0.080, 0.500, 0.025)),
        origin=Origin(xyz=(-0.025, 0.0, 0.135)),
        material=steel,
        name="axle_saddle",
    )
    frame.visual(
        Cylinder(radius=0.014, length=0.620),
        origin=Origin(xyz=(-0.055, 0.0, 0.130), rpy=(pi / 2.0, 0.0, 0.0)),
        material=zinc,
        name="wheel_axle",
    )

    for y in (-0.236, 0.236):
        frame.visual(
            Box((0.070, 0.022, 0.070)),
            origin=Origin(xyz=(0.036, y, 0.155)),
            material=steel,
            name=f"hinge_cheek_{0 if y < 0 else 1}",
        )
    frame.visual(
        Cylinder(radius=0.007, length=0.480),
        origin=Origin(xyz=(0.045, 0.0, 0.155), rpy=(pi / 2.0, 0.0, 0.0)),
        material=zinc,
        name="hinge_pin",
    )

    # Drop-down secondary shelf.  At q=0 it is tucked vertically just behind the
    # toe plate; positive rotation drops it forward into a broad platform.
    shelf = model.part("secondary_shelf")
    shelf.visual(
        Cylinder(radius=0.014, length=0.420),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=zinc,
        name="shelf_hinge_sleeve",
    )
    shelf.visual(
        Box((0.020, 0.420, 0.050)),
        origin=Origin(xyz=(0.020, 0.0, 0.039)),
        material=shelf_paint,
        name="hinge_leaf",
    )
    shelf.visual(
        Box((0.018, 0.420, 0.420)),
        origin=Origin(xyz=(0.020, 0.0, 0.218)),
        material=shelf_paint,
        name="shelf_panel",
    )
    for y in (-0.214, 0.214):
        shelf.visual(
            Box((0.032, 0.018, 0.420)),
            origin=Origin(xyz=(0.031, y, 0.218)),
            material=shelf_paint,
            name=f"side_lip_{0 if y < 0 else 1}",
        )
    shelf.visual(
        Box((0.035, 0.420, 0.026)),
        origin=Origin(xyz=(0.032, 0.0, 0.434)),
        material=shelf_paint,
        name="front_lip",
    )
    for y in (-0.125, 0.0, 0.125):
        shelf.visual(
            Box((0.007, 0.020, 0.320)),
            origin=Origin(xyz=(0.031, y, 0.245)),
            material=zinc,
            name=f"shelf_rib_{y:+.3f}",
        )

    model.articulation(
        "frame_to_secondary_shelf",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=shelf,
        origin=Origin(xyz=(0.045, 0.0, 0.155)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=2.0, lower=0.0, upper=pi / 2.0),
        motion_properties=MotionProperties(damping=0.05, friction=0.02),
    )

    # Two rubber transport wheels spin independently on the axle.
    wheel_mesh = WheelGeometry(
        0.092,
        0.052,
        rim=WheelRim(
            inner_radius=0.058,
            flange_height=0.006,
            flange_thickness=0.004,
            bead_seat_depth=0.003,
        ),
        hub=WheelHub(
            radius=0.030,
            width=0.044,
            cap_style="flat",
            bolt_pattern=BoltPattern(count=4, circle_diameter=0.038, hole_diameter=0.004),
        ),
        face=WheelFace(dish_depth=0.006, front_inset=0.002, rear_inset=0.002),
        spokes=WheelSpokes(style="straight", count=6, thickness=0.004, window_radius=0.012),
        bore=WheelBore(style="round", diameter=0.032),
    )
    tire_mesh = TireGeometry(
        0.130,
        0.060,
        inner_radius=0.086,
        carcass=TireCarcass(belt_width_ratio=0.72, sidewall_bulge=0.05),
        tread=TireTread(style="block", depth=0.006, count=18, land_ratio=0.58),
        grooves=(TireGroove(center_offset=0.0, width=0.006, depth=0.003),),
        sidewall=TireSidewall(style="rounded", bulge=0.035),
        shoulder=TireShoulder(width=0.006, radius=0.003),
    )

    for index, y in enumerate((-0.300, 0.300)):
        wheel = model.part(f"wheel_{index}")
        wheel.visual(
            mesh_from_geometry(tire_mesh, f"wheel_{index}_tire"),
            material=black,
            name="tire",
        )
        wheel.visual(
            mesh_from_geometry(wheel_mesh, f"wheel_{index}_rim"),
            material=zinc,
            name="rim",
        )
        wheel.visual(
            Cylinder(radius=0.017, length=0.052),
            origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
            material=zinc,
            name="axle_bushing",
        )
        model.articulation(
            f"frame_to_wheel_{index}",
            ArticulationType.CONTINUOUS,
            parent=frame,
            child=wheel,
            origin=Origin(xyz=(-0.055, y, 0.130), rpy=(0.0, 0.0, pi / 2.0)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=6.0, velocity=12.0),
            motion_properties=MotionProperties(damping=0.01, friction=0.01),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    shelf = object_model.get_part("secondary_shelf")
    shelf_joint = object_model.get_articulation("frame_to_secondary_shelf")

    ctx.allow_overlap(
        frame,
        shelf,
        elem_a="hinge_pin",
        elem_b="shelf_hinge_sleeve",
        reason="The visible hinge sleeve is represented as a solid proxy rotating around the captured steel pin.",
    )
    ctx.expect_within(
        frame,
        shelf,
        axes="xz",
        inner_elem="hinge_pin",
        outer_elem="shelf_hinge_sleeve",
        margin=0.002,
        name="hinge pin is captured inside the shelf sleeve",
    )
    ctx.expect_overlap(
        frame,
        shelf,
        axes="y",
        elem_a="hinge_pin",
        elem_b="shelf_hinge_sleeve",
        min_overlap=0.400,
        name="hinge sleeve spans most of the pin",
    )

    def element_x_center(part, elem_name: str) -> float | None:
        bounds = ctx.part_element_world_aabb(part, elem=elem_name)
        if bounds is None:
            return None
        lower, upper = bounds
        return (lower[0] + upper[0]) / 2.0

    # Stowed shelf is vertical and tucked just behind the primary toe plate.
    with ctx.pose({shelf_joint: 0.0}):
        ctx.expect_gap(
            shelf,
            frame,
            axis="z",
            positive_elem="shelf_panel",
            negative_elem="primary_toe_plate",
            min_gap=0.10,
            name="stowed shelf sits above the fixed toe plate",
        )
        ctx.expect_overlap(
            shelf,
            frame,
            axes="y",
            elem_a="shelf_panel",
            elem_b="primary_toe_plate",
            min_overlap=0.38,
            name="stowed shelf width matches the toe plate",
        )
        rest_panel_x = element_x_center(shelf, "shelf_panel")

    with ctx.pose({shelf_joint: pi / 2.0}):
        ctx.expect_gap(
            shelf,
            frame,
            axis="z",
            positive_elem="shelf_panel",
            negative_elem="primary_toe_plate",
            min_gap=0.06,
            name="dropped shelf clears the toe plate",
        )
        ctx.expect_overlap(
            shelf,
            frame,
            axes="x",
            elem_a="shelf_panel",
            elem_b="primary_toe_plate",
            min_overlap=0.25,
            name="dropped shelf projects over the toe plate",
        )
        dropped_panel_x = element_x_center(shelf, "shelf_panel")

    ctx.check(
        "secondary shelf drops forward",
        rest_panel_x is not None and dropped_panel_x is not None and dropped_panel_x > rest_panel_x + 0.10,
        details=f"rest_panel_x={rest_panel_x}, dropped_panel_x={dropped_panel_x}",
    )

    for index in (0, 1):
        wheel = object_model.get_part(f"wheel_{index}")
        wheel_joint = object_model.get_articulation(f"frame_to_wheel_{index}")
        ctx.allow_overlap(
            frame,
            wheel,
            elem_a="wheel_axle",
            elem_b="axle_bushing",
            reason="The spinning wheel uses a solid bearing-bushing proxy captured on the axle end.",
        )
        ctx.expect_within(
            frame,
            wheel,
            axes="xz",
            inner_elem="wheel_axle",
            outer_elem="axle_bushing",
            margin=0.003,
            name=f"wheel_{index} axle passes through the bushing",
        )
        ctx.expect_overlap(
            frame,
            wheel,
            axes="y",
            elem_a="wheel_axle",
            elem_b="axle_bushing",
            min_overlap=0.025,
            name=f"wheel_{index} bushing is retained on the axle",
        )
        ctx.expect_overlap(
            wheel,
            frame,
            axes="xz",
            elem_a="rim",
            elem_b="wheel_axle",
            min_overlap=0.02,
            name=f"wheel_{index} is centered on the axle",
        )
        before = ctx.part_world_position(wheel)
        with ctx.pose({wheel_joint: pi / 2.0}):
            after = ctx.part_world_position(wheel)
        ctx.check(
            f"wheel_{index} spins in place",
            before is not None and after is not None and max(abs(a - b) for a, b in zip(before, after)) < 1e-6,
            details=f"before={before}, after={after}",
        )

    return ctx.report()


object_model = build_object_model()
