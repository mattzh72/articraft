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
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    superellipse_profile,
)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _rounded_profile(width: float, depth: float, *, corner_segments: int = 8) -> list[tuple[float, float]]:
    radius = min(width, depth) * 0.22
    return rounded_rect_profile(width, depth, radius, corner_segments=corner_segments)


def _circle_profile(radius: float, *, segments: int = 48) -> list[tuple[float, float]]:
    diameter = radius * 2.0
    return superellipse_profile(diameter, diameter, exponent=2.0, segments=segments)


def _solid_profile_mesh(profile: list[tuple[float, float]], height: float, *, z0: float = 0.0):
    geometry = ExtrudeGeometry.from_z0(profile, height)
    if z0 != 0.0:
        geometry.translate(0.0, 0.0, z0)
    return geometry


def _ring_mesh(
    outer_profile: list[tuple[float, float]],
    inner_profile: list[tuple[float, float]],
    height: float,
    *,
    z0: float = 0.0,
):
    geometry = ExtrudeWithHolesGeometry(
        outer_profile,
        [inner_profile],
        height=height,
        center=False,
    )
    if z0 != 0.0:
        geometry.translate(0.0, 0.0, z0)
    return geometry


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="hand_sanitizer_pump_bottle")

    body_plastic = model.material("body_plastic", rgba=(0.72, 0.87, 0.84, 0.74))
    pump_white = model.material("pump_white", rgba=(0.95, 0.96, 0.97, 1.0))
    pump_gray = model.material("pump_gray", rgba=(0.78, 0.80, 0.82, 1.0))
    label_white = model.material("label_white", rgba=(0.97, 0.98, 0.98, 1.0))
    label_green = model.material("label_green", rgba=(0.27, 0.58, 0.43, 1.0))

    body_width = 0.086
    body_depth = 0.055
    wall_thickness = 0.0025
    base_thickness = 0.004
    straight_wall_height = 0.150
    shoulder_layer_height = 0.006

    body = model.part("body")
    body.inertial = Inertial.from_geometry(
        Box((0.090, 0.060, 0.212)),
        mass=0.42,
        origin=Origin(xyz=(0.0, 0.0, 0.106)),
    )

    outer_profile = _rounded_profile(body_width, body_depth)
    inner_profile = _rounded_profile(body_width - 2.0 * wall_thickness, body_depth - 2.0 * wall_thickness)

    body.visual(
        _save_mesh("sanitizer_body_base", _solid_profile_mesh(outer_profile, base_thickness)),
        material=body_plastic,
        name="body_base",
    )
    body.visual(
        _save_mesh(
            "sanitizer_body_wall",
            _ring_mesh(outer_profile, inner_profile, straight_wall_height, z0=base_thickness),
        ),
        material=body_plastic,
        name="body_shell",
    )

    shoulder_specs = [
        (0.082, 0.052, 0.076, 0.046),
        (0.074, 0.048, 0.066, 0.040),
        (0.062, 0.042, 0.052, 0.032),
        (0.049, 0.036, 0.038, 0.024),
        (0.036, 0.029, 0.026, 0.018),
    ]
    z_cursor = base_thickness + straight_wall_height
    for layer_index, (outer_w, outer_d, inner_w, inner_d) in enumerate(shoulder_specs):
        body.visual(
            _save_mesh(
                f"sanitizer_shoulder_layer_{layer_index}",
                _ring_mesh(
                    _rounded_profile(outer_w, outer_d, corner_segments=7),
                    _rounded_profile(inner_w, inner_d, corner_segments=7),
                    shoulder_layer_height,
                    z0=z_cursor,
                ),
            ),
            material=body_plastic,
            name=f"shoulder_layer_{layer_index}",
        )
        z_cursor += shoulder_layer_height

    neck_z0 = z_cursor
    body.visual(
        _save_mesh(
            "sanitizer_neck_shell",
            _ring_mesh(
                _circle_profile(0.0155),
                _circle_profile(0.0092),
                0.018,
                z0=neck_z0,
            ),
        ),
        material=body_plastic,
        name="neck_shell",
    )
    body.visual(
        _save_mesh(
            "sanitizer_pump_collar",
            _ring_mesh(
                _circle_profile(0.0215),
                _circle_profile(0.0158),
                0.010,
                z0=neck_z0 - 0.002,
            ),
        ),
        material=pump_white,
        name="pump_collar",
    )
    body.visual(
        _save_mesh(
            "sanitizer_guide_sleeve",
            _ring_mesh(
                _circle_profile(0.0112),
                _circle_profile(0.0072),
                0.022,
                z0=neck_z0 + 0.010,
            ),
        ),
        material=pump_white,
        name="guide_sleeve",
    )
    body.visual(
        Box((0.050, 0.0012, 0.085)),
        origin=Origin(xyz=(0.0, body_depth * 0.5 - 0.0003, 0.083)),
        material=label_white,
        name="label_panel",
    )
    body.visual(
        Box((0.028, 0.0014, 0.020)),
        origin=Origin(xyz=(0.0, body_depth * 0.5 - 0.0002, 0.055)),
        material=label_green,
        name="label_band",
    )

    plunger = model.part("plunger")
    plunger.inertial = Inertial.from_geometry(
        Box((0.018, 0.018, 0.150)),
        mass=0.05,
        origin=Origin(xyz=(0.0, 0.0, -0.050)),
    )
    plunger.visual(
        Cylinder(radius=0.0064, length=0.140),
        origin=Origin(xyz=(0.0, 0.0, -0.064)),
        material=pump_gray,
        name="plunger_stem",
    )
    plunger.visual(
        Cylinder(radius=0.0088, length=0.026),
        origin=Origin(xyz=(0.0, 0.0, 0.013)),
        material=pump_white,
        name="head_post",
    )

    nozzle = model.part("nozzle")
    nozzle.inertial = Inertial.from_geometry(
        Box((0.066, 0.030, 0.034)),
        mass=0.06,
        origin=Origin(xyz=(0.014, 0.0, 0.016)),
    )
    actuator_pad = _save_mesh(
        "sanitizer_actuator_pad",
        _solid_profile_mesh(_rounded_profile(0.070, 0.027, corner_segments=8), 0.012),
    )
    nozzle.visual(
        actuator_pad,
        origin=Origin(xyz=(-0.004, 0.0, 0.010)),
        material=pump_white,
        name="actuator_pad",
    )
    nozzle.visual(
        Cylinder(radius=0.013, length=0.015),
        origin=Origin(xyz=(0.0, 0.0, 0.0075)),
        material=pump_white,
        name="nozzle_barrel",
    )
    nozzle.visual(
        Cylinder(radius=0.006, length=0.034),
        origin=Origin(xyz=(0.020, 0.0, 0.014), rpy=(0.0, pi / 2.0, 0.0)),
        material=pump_white,
        name="spout_body",
    )
    nozzle.visual(
        Cylinder(radius=0.0042, length=0.012),
        origin=Origin(xyz=(0.043, 0.0, 0.014), rpy=(0.0, pi / 2.0, 0.0)),
        material=pump_white,
        name="spout_tip",
    )
    nozzle.visual(
        Cylinder(radius=0.009, length=0.010),
        origin=Origin(xyz=(0.006, 0.0, 0.014), rpy=(0.0, pi / 2.0, 0.0)),
        material=pump_white,
        name="spout_root",
    )

    model.articulation(
        "body_to_plunger",
        ArticulationType.PRISMATIC,
        parent=body,
        child=plunger,
        origin=Origin(xyz=(0.0, 0.0, neck_z0 + 0.028)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=0.20,
            lower=0.0,
            upper=0.018,
        ),
    )
    model.articulation(
        "plunger_to_nozzle",
        ArticulationType.REVOLUTE,
        parent=plunger,
        child=nozzle,
        origin=Origin(xyz=(0.0, 0.0, 0.026)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=2.5,
            lower=0.0,
            upper=pi / 2.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    plunger = object_model.get_part("plunger")
    nozzle = object_model.get_part("nozzle")
    slide = object_model.get_articulation("body_to_plunger")
    swivel = object_model.get_articulation("plunger_to_nozzle")

    with ctx.pose({slide: 0.0, swivel: 0.0}):
        ctx.expect_overlap(
            plunger,
            body,
            axes="z",
            elem_a="plunger_stem",
            elem_b="guide_sleeve",
            min_overlap=0.020,
            name="resting plunger stem is deeply retained in the guide sleeve",
        )
        ctx.expect_contact(
            plunger,
            nozzle,
            elem_a="head_post",
            elem_b="nozzle_barrel",
            contact_tol=1e-4,
            name="nozzle body sits on the plunger post",
        )
        ctx.expect_gap(
            nozzle,
            body,
            axis="z",
            min_gap=0.010,
            max_gap=0.060,
            name="nozzle assembly rides above the bottle neck",
        )
        ctx.expect_overlap(
            nozzle,
            body,
            axes="y",
            min_overlap=0.020,
            name="nozzle remains centered across the bottle depth",
        )

    body_aabb = ctx.part_world_aabb(body)
    if body_aabb is not None:
        body_size = tuple(body_aabb[1][axis] - body_aabb[0][axis] for axis in range(3))
        ctx.check(
            "body reads as a tall rectangular bottle",
            0.075 <= body_size[0] <= 0.095
            and 0.048 <= body_size[1] <= 0.060
            and 0.195 <= body_size[2] <= 0.220
            and body_size[2] > body_size[0] * 2.2,
            details=f"size={body_size}",
        )
    else:
        ctx.fail("body reads as a tall rectangular bottle", "missing body AABB")

    rest_nozzle_pos = ctx.part_world_position(nozzle)
    pressed_upper = slide.motion_limits.upper if slide.motion_limits is not None else None
    pressed_nozzle_pos = None
    if pressed_upper is None:
        ctx.fail("plunger has a downward travel range", "missing prismatic upper limit")
    else:
        with ctx.pose({slide: pressed_upper, swivel: 0.0}):
            pressed_nozzle_pos = ctx.part_world_position(nozzle)
            ctx.expect_overlap(
                plunger,
                body,
                axes="z",
                elem_a="plunger_stem",
                elem_b="guide_sleeve",
                min_overlap=0.005,
                name="pressed plunger still remains engaged in the guide sleeve",
            )
        ctx.check(
            "plunger depresses downward along the bottle axis",
            rest_nozzle_pos is not None
            and pressed_nozzle_pos is not None
            and pressed_nozzle_pos[2] < rest_nozzle_pos[2] - 0.014,
            details=f"rest={rest_nozzle_pos}, pressed={pressed_nozzle_pos}",
        )

    def _aabb_center(aabb):
        if aabb is None:
            return None
        return tuple((aabb[0][axis] + aabb[1][axis]) * 0.5 for axis in range(3))

    with ctx.pose({slide: 0.0, swivel: 0.0}):
        forward_tip = _aabb_center(ctx.part_element_world_aabb(nozzle, elem="spout_tip"))
    with ctx.pose({slide: 0.0, swivel: pi / 2.0}):
        locked_tip = _aabb_center(ctx.part_element_world_aabb(nozzle, elem="spout_tip"))
        ctx.expect_gap(
            nozzle,
            body,
            axis="z",
            min_gap=0.010,
            name="locked nozzle still clears the bottle top",
        )
    ctx.check(
        "nozzle rotates from forward dispense to side-locked pose",
        forward_tip is not None
        and locked_tip is not None
        and forward_tip[0] > 0.035
        and abs(forward_tip[1]) < 0.008
        and locked_tip[1] > 0.035
        and locked_tip[0] < 0.015,
        details=f"forward_tip={forward_tip}, locked_tip={locked_tip}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
