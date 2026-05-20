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
    CylinderGeometry,
    Inertial,
    MotionLimits,
    Origin,
    SphereGeometry,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    section_loft,
)


def _dovetail_rail_mesh(
    *,
    length: float,
    top_width: float,
    base_width: float,
    height: float,
    axis: str,
    name: str,
):
    if axis == "x":
        start = (
            (-length / 2.0, -base_width / 2.0, 0.0),
            (-length / 2.0, base_width / 2.0, 0.0),
            (-length / 2.0, top_width / 2.0, height),
            (-length / 2.0, -top_width / 2.0, height),
        )
        end = (
            (length / 2.0, -base_width / 2.0, 0.0),
            (length / 2.0, base_width / 2.0, 0.0),
            (length / 2.0, top_width / 2.0, height),
            (length / 2.0, -top_width / 2.0, height),
        )
    elif axis == "y":
        start = (
            (-base_width / 2.0, -length / 2.0, 0.0),
            (base_width / 2.0, -length / 2.0, 0.0),
            (top_width / 2.0, -length / 2.0, height),
            (-top_width / 2.0, -length / 2.0, height),
        )
        end = (
            (-base_width / 2.0, length / 2.0, 0.0),
            (base_width / 2.0, length / 2.0, 0.0),
            (top_width / 2.0, length / 2.0, height),
            (-top_width / 2.0, length / 2.0, height),
        )
    else:
        raise ValueError(f"Unsupported rail axis: {axis}")

    return mesh_from_geometry(section_loft([start, end]), name)


def _handwheel_mesh(*, axis: str, radius: float, tube: float, name: str):
    wheel = TorusGeometry(radius=radius, tube=tube, radial_segments=18, tubular_segments=40)
    if axis == "x":
        wheel.rotate_y(math.pi / 2.0)
    elif axis == "y":
        wheel.rotate_x(math.pi / 2.0)
    else:
        raise ValueError(f"Unsupported wheel axis: {axis}")

    spoke = CylinderGeometry(radius=0.0024, height=radius * 0.92, radial_segments=16).translate(
        0.0,
        0.0,
        radius * 0.46,
    )
    knob = SphereGeometry(0.005).translate(0.0, 0.0, radius)
    wheel.merge(spoke)
    wheel.merge(knob)
    return mesh_from_geometry(wheel, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cross_slide_drill_press_vise")

    cast_iron = model.material("cast_iron", rgba=(0.22, 0.28, 0.34, 1.0))
    machined_steel = model.material("machined_steel", rgba=(0.74, 0.76, 0.78, 1.0))
    dark_oxide = model.material("dark_oxide", rgba=(0.16, 0.17, 0.18, 1.0))
    jaw_face = model.material("jaw_face", rgba=(0.85, 0.86, 0.88, 1.0))

    x_rail_mesh = _dovetail_rail_mesh(
        length=0.23,
        top_width=0.014,
        base_width=0.024,
        height=0.014,
        axis="x",
        name="x_dovetail_rail",
    )
    y_rail_mesh = _dovetail_rail_mesh(
        length=0.16,
        top_width=0.012,
        base_width=0.022,
        height=0.012,
        axis="y",
        name="y_dovetail_rail",
    )
    x_wheel_mesh = _handwheel_mesh(axis="x", radius=0.024, tube=0.0026, name="x_handwheel")
    y_wheel_mesh = _handwheel_mesh(axis="y", radius=0.022, tube=0.0024, name="y_handwheel")

    base = model.part("base")
    base.visual(
        Box((0.30, 0.18, 0.020)),
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        material=cast_iron,
        name="base_plate",
    )
    base.visual(
        Box((0.26, 0.11, 0.016)),
        origin=Origin(xyz=(0.0, 0.0, 0.028)),
        material=cast_iron,
        name="x_way_plinth",
    )
    base.visual(
        x_rail_mesh,
        origin=Origin(xyz=(0.0, -0.028, 0.036)),
        material=machined_steel,
        name="x_rail_left",
    )
    base.visual(
        x_rail_mesh,
        origin=Origin(xyz=(0.0, 0.028, 0.036)),
        material=machined_steel,
        name="x_rail_right",
    )
    base.visual(
        Box((0.012, 0.038, 0.009)),
        origin=Origin(xyz=(0.136, 0.0, 0.0245)),
        material=cast_iron,
        name="x_bearing_pedestal_right",
    )
    base.visual(
        Box((0.008, 0.006, 0.012)),
        origin=Origin(xyz=(0.141, -0.011, 0.034)),
        material=cast_iron,
        name="x_bearing_cheek_right_lower",
    )
    base.visual(
        Box((0.008, 0.006, 0.012)),
        origin=Origin(xyz=(0.141, 0.011, 0.034)),
        material=cast_iron,
        name="x_bearing_cheek_right_upper",
    )
    base.visual(
        Box((0.012, 0.030, 0.009)),
        origin=Origin(xyz=(-0.136, 0.0, 0.0245)),
        material=cast_iron,
        name="x_anchor_pedestal_left",
    )
    base.visual(
        Box((0.006, 0.006, 0.010)),
        origin=Origin(xyz=(-0.139, -0.010, 0.033)),
        material=cast_iron,
        name="x_anchor_cheek_left_lower",
    )
    base.visual(
        Box((0.006, 0.006, 0.010)),
        origin=Origin(xyz=(-0.139, 0.010, 0.033)),
        material=cast_iron,
        name="x_anchor_cheek_left_upper",
    )
    base.inertial = Inertial.from_geometry(
        Box((0.30, 0.18, 0.060)),
        mass=6.8,
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
    )

    saddle = model.part("saddle")
    saddle.visual(
        Box((0.20, 0.14, 0.022)),
        origin=Origin(xyz=(0.0, 0.0, 0.011)),
        material=cast_iron,
        name="saddle_slide_plate",
    )
    saddle.visual(
        Box((0.20, 0.010, 0.028)),
        origin=Origin(xyz=(0.0, -0.048, 0.004)),
        material=cast_iron,
        name="saddle_keeper_left",
    )
    saddle.visual(
        Box((0.20, 0.010, 0.028)),
        origin=Origin(xyz=(0.0, 0.048, 0.004)),
        material=cast_iron,
        name="saddle_keeper_right",
    )
    saddle.visual(
        Box((0.12, 0.16, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.028)),
        material=cast_iron,
        name="y_way_plinth",
    )
    saddle.visual(
        y_rail_mesh,
        origin=Origin(xyz=(-0.030, 0.0, 0.034)),
        material=machined_steel,
        name="y_rail_left",
    )
    saddle.visual(
        y_rail_mesh,
        origin=Origin(xyz=(0.030, 0.0, 0.034)),
        material=machined_steel,
        name="y_rail_right",
    )
    saddle.visual(
        Box((0.050, 0.016, 0.018)),
        origin=Origin(xyz=(0.0, -0.088, 0.031)),
        material=cast_iron,
        name="y_bearing_pedestal",
    )
    saddle.visual(
        Box((0.006, 0.016, 0.016)),
        origin=Origin(xyz=(-0.014, -0.088, 0.046)),
        material=cast_iron,
        name="y_bearing_cheek_left",
    )
    saddle.visual(
        Box((0.006, 0.016, 0.016)),
        origin=Origin(xyz=(0.014, -0.088, 0.046)),
        material=cast_iron,
        name="y_bearing_cheek_right",
    )
    saddle.inertial = Inertial.from_geometry(
        Box((0.20, 0.16, 0.060)),
        mass=3.4,
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
    )

    jaw_body = model.part("jaw_body")
    jaw_body.visual(
        Box((0.18, 0.13, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=cast_iron,
        name="jaw_slide_plate",
    )
    jaw_body.visual(
        Box((0.010, 0.13, 0.026)),
        origin=Origin(xyz=(-0.049, 0.0, 0.002)),
        material=cast_iron,
        name="jaw_keeper_left",
    )
    jaw_body.visual(
        Box((0.010, 0.13, 0.026)),
        origin=Origin(xyz=(0.049, 0.0, 0.002)),
        material=cast_iron,
        name="jaw_keeper_right",
    )
    jaw_body.visual(
        Box((0.16, 0.086, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.027)),
        material=cast_iron,
        name="vise_bed",
    )
    jaw_body.visual(
        Box((0.012, 0.078, 0.028)),
        origin=Origin(xyz=(-0.074, 0.0, 0.050)),
        material=cast_iron,
        name="vise_wall_left",
    )
    jaw_body.visual(
        Box((0.012, 0.078, 0.028)),
        origin=Origin(xyz=(0.074, 0.0, 0.050)),
        material=cast_iron,
        name="vise_wall_right",
    )
    jaw_body.visual(
        Box((0.16, 0.012, 0.028)),
        origin=Origin(xyz=(0.0, 0.034, 0.050)),
        material=cast_iron,
        name="rear_jaw_block",
    )
    jaw_body.visual(
        Box((0.16, 0.012, 0.024)),
        origin=Origin(xyz=(0.0, -0.034, 0.048)),
        material=cast_iron,
        name="front_jaw_block",
    )
    jaw_body.visual(
        Box((0.150, 0.004, 0.022)),
        origin=Origin(xyz=(0.0, 0.026, 0.049)),
        material=jaw_face,
        name="rear_jaw_face",
    )
    jaw_body.visual(
        Box((0.150, 0.004, 0.020)),
        origin=Origin(xyz=(0.0, -0.026, 0.047)),
        material=jaw_face,
        name="front_jaw_face",
    )
    jaw_body.inertial = Inertial.from_geometry(
        Box((0.18, 0.13, 0.072)),
        mass=3.8,
        origin=Origin(xyz=(0.0, 0.0, 0.036)),
    )

    x_handle = model.part("x_handle")
    x_handle.visual(
        Cylinder(radius=0.0055, length=0.022),
        origin=Origin(xyz=(-0.004, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_oxide,
        name="x_leadscrew_stub",
    )
    x_handle.visual(
        Cylinder(radius=0.010, length=0.020),
        origin=Origin(xyz=(0.012, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_oxide,
        name="x_hub",
    )
    x_handle.visual(
        x_wheel_mesh,
        origin=Origin(xyz=(0.016, 0.0, 0.0)),
        material=machined_steel,
        name="x_handwheel_rim",
    )
    x_handle.inertial = Inertial.from_geometry(
        Box((0.055, 0.050, 0.055)),
        mass=0.35,
        origin=Origin(xyz=(0.008, 0.0, 0.0)),
    )

    y_handle = model.part("y_handle")
    y_handle.visual(
        Cylinder(radius=0.0050, length=0.040),
        origin=Origin(xyz=(0.0, 0.006, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_oxide,
        name="y_leadscrew_stub",
    )
    y_handle.visual(
        Cylinder(radius=0.009, length=0.022),
        origin=Origin(xyz=(0.0, -0.012, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_oxide,
        name="y_hub",
    )
    y_handle.visual(
        y_wheel_mesh,
        origin=Origin(xyz=(0.0, -0.012, 0.0)),
        material=machined_steel,
        name="y_handwheel_rim",
    )
    y_handle.inertial = Inertial.from_geometry(
        Box((0.050, 0.050, 0.050)),
        mass=0.30,
        origin=Origin(xyz=(0.0, -0.008, 0.0)),
    )

    model.articulation(
        "base_to_saddle",
        ArticulationType.PRISMATIC,
        parent=base,
        child=saddle,
        origin=Origin(xyz=(0.0, 0.0, 0.050)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=180.0,
            velocity=0.18,
            lower=-0.055,
            upper=0.055,
        ),
    )
    model.articulation(
        "saddle_to_jaw_body",
        ArticulationType.PRISMATIC,
        parent=saddle,
        child=jaw_body,
        origin=Origin(xyz=(0.0, 0.0, 0.046)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=160.0,
            velocity=0.16,
            lower=-0.038,
            upper=0.038,
        ),
    )
    model.articulation(
        "base_to_x_handle",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=x_handle,
        origin=Origin(xyz=(0.148, 0.0, 0.034)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=12.0),
    )
    model.articulation(
        "saddle_to_y_handle",
        ArticulationType.CONTINUOUS,
        parent=saddle,
        child=y_handle,
        origin=Origin(xyz=(0.0, -0.088, 0.046)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=12.0),
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
    base = object_model.get_part("base")
    saddle = object_model.get_part("saddle")
    jaw_body = object_model.get_part("jaw_body")
    x_handle = object_model.get_part("x_handle")
    y_handle = object_model.get_part("y_handle")

    x_slide = object_model.get_articulation("base_to_saddle")
    y_slide = object_model.get_articulation("saddle_to_jaw_body")
    x_wheel = object_model.get_articulation("base_to_x_handle")
    y_wheel = object_model.get_articulation("saddle_to_y_handle")

    with ctx.pose({x_slide: 0.0, y_slide: 0.0}):
        ctx.expect_gap(
            saddle,
            base,
            axis="z",
            positive_elem="saddle_slide_plate",
            negative_elem="x_rail_left",
            min_gap=0.0,
            max_gap=0.001,
            name="saddle sits on the left X dovetail rail",
        )
        ctx.expect_gap(
            saddle,
            base,
            axis="z",
            positive_elem="saddle_slide_plate",
            negative_elem="x_rail_right",
            min_gap=0.0,
            max_gap=0.001,
            name="saddle sits on the right X dovetail rail",
        )
        ctx.expect_gap(
            jaw_body,
            saddle,
            axis="z",
            positive_elem="jaw_slide_plate",
            negative_elem="y_rail_left",
            min_gap=0.0,
            max_gap=0.001,
            name="jaw body sits on the left Y rail",
        )
        ctx.expect_gap(
            jaw_body,
            saddle,
            axis="z",
            positive_elem="jaw_slide_plate",
            negative_elem="y_rail_right",
            min_gap=0.0,
            max_gap=0.001,
            name="jaw body sits on the right Y rail",
        )
        ctx.expect_overlap(
            saddle,
            base,
            axes="xy",
            min_overlap=0.12,
            name="saddle remains broadly supported on the base",
        )
        ctx.expect_overlap(
            jaw_body,
            saddle,
            axes="xy",
            min_overlap=0.09,
            name="jaw body remains broadly supported on the saddle",
        )

    x_rest = ctx.part_world_position(saddle)
    with ctx.pose({x_slide: 0.055}):
        x_shifted = ctx.part_world_position(saddle)
        ctx.expect_overlap(
            saddle,
            base,
            axes="x",
            min_overlap=0.14,
            name="saddle retains insertion on X ways at full travel",
        )
        ctx.expect_overlap(
            saddle,
            base,
            axes="y",
            min_overlap=0.12,
            name="saddle stays laterally captured on the base ways",
        )
        ctx.expect_gap(
            x_handle,
            saddle,
            axis="x",
            positive_elem="x_handwheel_rim",
            min_gap=0.003,
            name="X handwheel clears the saddle at full X travel",
        )
    ctx.check(
        "saddle travels in +X at the X-slide upper limit",
        x_rest is not None and x_shifted is not None and x_shifted[0] > x_rest[0] + 0.04,
        details=f"rest={x_rest}, shifted={x_shifted}",
    )

    y_rest = ctx.part_world_position(jaw_body)
    with ctx.pose({y_slide: 0.038}):
        y_shifted = ctx.part_world_position(jaw_body)
        ctx.expect_overlap(
            jaw_body,
            saddle,
            axes="y",
            min_overlap=0.05,
            name="jaw body retains insertion on Y ways at full travel",
        )
        ctx.expect_overlap(
            jaw_body,
            saddle,
            axes="x",
            min_overlap=0.10,
            name="jaw body stays centered between the Y ways",
        )
        ctx.expect_gap(
            jaw_body,
            y_handle,
            axis="y",
            min_gap=0.03,
            name="Y handwheel stays in front of the moving jaw body",
        )
    ctx.check(
        "jaw body travels in +Y at the Y-slide upper limit",
        y_rest is not None and y_shifted is not None and y_shifted[1] > y_rest[1] + 0.025,
        details=f"rest={y_rest}, shifted={y_shifted}",
    )

    x_handle_rest = ctx.part_world_position(x_handle)
    with ctx.pose({x_wheel: math.pi}):
        x_handle_turned = ctx.part_world_position(x_handle)
    ctx.check(
        "X handwheel rotates in place on the base screw axis",
        x_handle_rest == x_handle_turned,
        details=f"rest={x_handle_rest}, turned={x_handle_turned}",
    )

    y_handle_rest = ctx.part_world_position(y_handle)
    with ctx.pose({y_wheel: math.pi / 2.0}):
        y_handle_turned = ctx.part_world_position(y_handle)
    ctx.check(
        "Y handwheel rotates in place on the saddle screw axis",
        y_handle_rest == y_handle_turned,
        details=f"rest={y_handle_rest}, turned={y_handle_turned}",
    )

    ctx.check(
        "both crank handles are continuous articulations",
        x_wheel.articulation_type == ArticulationType.CONTINUOUS
        and y_wheel.articulation_type == ArticulationType.CONTINUOUS
        and x_wheel.motion_limits is not None
        and y_wheel.motion_limits is not None
        and x_wheel.motion_limits.lower is None
        and x_wheel.motion_limits.upper is None
        and y_wheel.motion_limits.lower is None
        and y_wheel.motion_limits.upper is None,
        details=(
            f"x_type={x_wheel.articulation_type}, x_limits={x_wheel.motion_limits}, "
            f"y_type={y_wheel.articulation_type}, y_limits={y_wheel.motion_limits}"
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
