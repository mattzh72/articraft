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
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _ring_shell_mesh(name: str, *, outer_radius: float, inner_radius: float, width: float):
    shell = LatheGeometry.from_shell_profiles(
        [
            (outer_radius * 0.96, -0.5 * width),
            (outer_radius, -0.22 * width),
            (outer_radius, 0.22 * width),
            (outer_radius * 0.96, 0.5 * width),
        ],
        [
            (inner_radius, -0.53 * width),
            (inner_radius, 0.53 * width),
        ],
        segments=56,
        start_cap="flat",
        end_cap="flat",
    ).rotate_x(-pi / 2.0)
    return mesh_from_geometry(shell, name)


def _add_optical_side(
    part,
    *,
    sign: float,
    armor,
    metal,
    glass,
    rubber,
) -> None:
    y_axis = Origin(rpy=(-pi / 2.0, 0.0, 0.0))

    part.visual(
        Box((0.050, 0.064, 0.050)),
        material=armor,
        name="upper_housing",
    )
    part.visual(
        Box((0.058, 0.066, 0.050)),
        origin=Origin(xyz=(sign * 0.042, 0.040, -0.035)),
        material=armor,
        name="lower_housing",
    )
    part.visual(
        Box((0.024, 0.056, 0.082)),
        origin=Origin(xyz=(sign * 0.022, 0.020, -0.018)),
        material=armor,
        name="porro_web",
    )
    part.visual(
        Box((0.018, 0.056, 0.024)),
        origin=Origin(xyz=(-sign * 0.010, -0.002, -0.012)),
        material=metal,
        name="inner_shoulder",
    )

    part.visual(
        Cylinder(radius=0.031, length=0.082),
        origin=Origin(
            xyz=(sign * 0.048, 0.098, -0.035),
            rpy=y_axis.rpy,
        ),
        material=armor,
        name="objective_shell",
    )
    part.visual(
        Cylinder(radius=0.034, length=0.020),
        origin=Origin(
            xyz=(sign * 0.048, 0.056, -0.035),
            rpy=y_axis.rpy,
        ),
        material=rubber,
        name="objective_armor_band",
    )
    part.visual(
        Cylinder(radius=0.026, length=0.004),
        origin=Origin(
            xyz=(sign * 0.048, 0.138, -0.035),
            rpy=y_axis.rpy,
        ),
        material=glass,
        name="objective_lens",
    )

    part.visual(
        Cylinder(radius=0.015, length=0.048),
        origin=Origin(
            xyz=(-sign * 0.012, -0.050, 0.000),
            rpy=y_axis.rpy,
        ),
        material=metal,
        name="eyepiece_barrel",
    )
    part.visual(
        Cylinder(radius=0.023, length=0.028),
        origin=Origin(
            xyz=(-sign * 0.012, -0.083, 0.000),
            rpy=y_axis.rpy,
        ),
        material=rubber,
        name="eyecup",
    )
    part.visual(
        Cylinder(radius=0.011, length=0.003),
        origin=Origin(
            xyz=(-sign * 0.012, -0.096, 0.000),
            rpy=y_axis.rpy,
        ),
        material=glass,
        name="eyepiece_lens",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="porro_field_binocular")

    armor = model.material("armor", rgba=(0.12, 0.12, 0.13, 1.0))
    metal = model.material("metal", rgba=(0.27, 0.29, 0.31, 1.0))
    wheel_rubber = model.material("wheel_rubber", rgba=(0.08, 0.08, 0.09, 1.0))
    glass = model.material("glass", rgba=(0.34, 0.48, 0.62, 0.82))
    knurl_metal = model.material("knurl_metal", rgba=(0.22, 0.23, 0.25, 1.0))

    bridge = model.part("bridge")
    bridge.visual(
        Box((0.044, 0.046, 0.028)),
        origin=Origin(xyz=(0.0, 0.000, 0.000)),
        material=armor,
        name="bridge_core",
    )
    bridge.visual(
        Box((0.038, 0.022, 0.020)),
        origin=Origin(xyz=(0.0, -0.018, -0.004)),
        material=armor,
        name="rear_bridge_strap",
    )
    bridge.visual(
        Box((0.010, 0.018, 0.048)),
        origin=Origin(xyz=(0.017, 0.020, -0.030)),
        material=metal,
        name="focus_cheek_left",
    )
    bridge.visual(
        Box((0.010, 0.018, 0.048)),
        origin=Origin(xyz=(-0.017, 0.020, -0.030)),
        material=metal,
        name="focus_cheek_right",
    )
    bridge.inertial = Inertial.from_geometry(
        Box((0.11, 0.08, 0.08)),
        mass=0.42,
        origin=Origin(xyz=(0.0, -0.002, -0.010)),
    )

    left_body = model.part("left_body")
    _add_optical_side(
        left_body,
        sign=1.0,
        armor=armor,
        metal=metal,
        glass=glass,
        rubber=wheel_rubber,
    )
    left_body.inertial = Inertial.from_geometry(
        Box((0.13, 0.25, 0.11)),
        mass=0.38,
        origin=Origin(xyz=(0.030, 0.020, -0.010)),
    )

    right_body = model.part("right_body")
    _add_optical_side(
        right_body,
        sign=-1.0,
        armor=armor,
        metal=metal,
        glass=glass,
        rubber=wheel_rubber,
    )
    right_body.inertial = Inertial.from_geometry(
        Box((0.13, 0.25, 0.11)),
        mass=0.38,
        origin=Origin(xyz=(-0.030, 0.020, -0.010)),
    )

    focus_wheel = model.part("focus_wheel")
    focus_wheel.visual(
        Cylinder(radius=0.020, length=0.024),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=wheel_rubber,
        name="focus_tread",
    )
    focus_wheel.visual(
        Cylinder(radius=0.014, length=0.028),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=knurl_metal,
        name="focus_core",
    )
    focus_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.020, length=0.024),
        mass=0.04,
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
    )

    diopter_ring = model.part("right_diopter_ring")
    diopter_ring.visual(
        _ring_shell_mesh(
            "right_diopter_ring_shell",
            outer_radius=0.0215,
            inner_radius=0.0162,
            width=0.010,
        ),
        material=wheel_rubber,
        name="diopter_ring_shell",
    )
    diopter_ring.inertial = Inertial.from_geometry(
        Box((0.045, 0.012, 0.045)),
        mass=0.01,
    )

    model.articulation(
        "bridge_to_left_body",
        ArticulationType.FIXED,
        parent=bridge,
        child=left_body,
        origin=Origin(xyz=(0.047, 0.000, 0.000)),
    )
    model.articulation(
        "bridge_to_right_body",
        ArticulationType.FIXED,
        parent=bridge,
        child=right_body,
        origin=Origin(xyz=(-0.047, 0.000, 0.000)),
    )
    model.articulation(
        "bridge_to_focus_wheel",
        ArticulationType.CONTINUOUS,
        parent=bridge,
        child=focus_wheel,
        origin=Origin(xyz=(0.0, 0.020, -0.030)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.5, velocity=12.0),
    )
    model.articulation(
        "right_body_to_diopter_ring",
        ArticulationType.CONTINUOUS,
        parent=right_body,
        child=diopter_ring,
        origin=Origin(xyz=(0.012, -0.064, 0.000)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=0.12, velocity=15.0),
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

    bridge = object_model.get_part("bridge")
    left_body = object_model.get_part("left_body")
    right_body = object_model.get_part("right_body")
    focus_wheel = object_model.get_part("focus_wheel")
    diopter_ring = object_model.get_part("right_diopter_ring")

    focus_joint = object_model.get_articulation("bridge_to_focus_wheel")
    diopter_joint = object_model.get_articulation("right_body_to_diopter_ring")

    ctx.expect_contact(
        left_body,
        bridge,
        contact_tol=0.001,
        name="left optical body is mounted to the bridge",
    )
    ctx.expect_contact(
        right_body,
        bridge,
        contact_tol=0.001,
        name="right optical body is mounted to the bridge",
    )
    ctx.expect_contact(
        focus_wheel,
        bridge,
        contact_tol=0.001,
        name="focus wheel sits in the bridge saddle",
    )
    ctx.expect_overlap(
        diopter_ring,
        right_body,
        axes="xz",
        elem_b="eyepiece_barrel",
        min_overlap=0.020,
        name="diopter ring wraps around the right eyepiece axis",
    )

    ctx.check(
        "focus wheel articulation is continuous about the lateral axle",
        focus_joint.articulation_type == ArticulationType.CONTINUOUS
        and focus_joint.axis == (1.0, 0.0, 0.0)
        and focus_joint.motion_limits is not None
        and focus_joint.motion_limits.lower is None
        and focus_joint.motion_limits.upper is None,
        details=f"type={focus_joint.articulation_type}, axis={focus_joint.axis}, limits={focus_joint.motion_limits}",
    )
    ctx.check(
        "diopter ring articulation is continuous about the eyepiece axis",
        diopter_joint.articulation_type == ArticulationType.CONTINUOUS
        and diopter_joint.axis == (0.0, -1.0, 0.0)
        and diopter_joint.motion_limits is not None
        and diopter_joint.motion_limits.lower is None
        and diopter_joint.motion_limits.upper is None,
        details=f"type={diopter_joint.articulation_type}, axis={diopter_joint.axis}, limits={diopter_joint.motion_limits}",
    )

    left_objective = ctx.part_element_world_aabb(left_body, elem="objective_shell")
    right_objective = ctx.part_element_world_aabb(right_body, elem="objective_shell")
    left_eyepiece = ctx.part_element_world_aabb(left_body, elem="eyepiece_barrel")
    right_eyepiece = ctx.part_element_world_aabb(right_body, elem="eyepiece_barrel")
    ring_aabb = ctx.part_world_aabb(diopter_ring)

    def _center_x(aabb):
        return None if aabb is None else 0.5 * (aabb[0][0] + aabb[1][0])

    def _center_z(aabb):
        return None if aabb is None else 0.5 * (aabb[0][2] + aabb[1][2])

    objective_spread = None
    eyepiece_spread = None
    if left_objective and right_objective and left_eyepiece and right_eyepiece:
        objective_spread = abs(_center_x(left_objective) - _center_x(right_objective))
        eyepiece_spread = abs(_center_x(left_eyepiece) - _center_x(right_eyepiece))

    ctx.check(
        "porro layout keeps objective tubes farther apart than eyepieces",
        objective_spread is not None
        and eyepiece_spread is not None
        and objective_spread > eyepiece_spread + 0.08,
        details=f"objective_spread={objective_spread}, eyepiece_spread={eyepiece_spread}",
    )

    right_eyepiece_center_x = _center_x(right_eyepiece)
    right_eyepiece_center_z = _center_z(right_eyepiece)
    ring_center_x = _center_x(ring_aabb)
    ring_center_z = _center_z(ring_aabb)
    ctx.check(
        "diopter ring remains centered on the right eyepiece tube",
        right_eyepiece_center_x is not None
        and right_eyepiece_center_z is not None
        and ring_center_x is not None
        and ring_center_z is not None
        and abs(right_eyepiece_center_x - ring_center_x) < 0.003
        and abs(right_eyepiece_center_z - ring_center_z) < 0.003,
        details=(
            f"eyepiece_center=({right_eyepiece_center_x}, {right_eyepiece_center_z}), "
            f"ring_center=({ring_center_x}, {ring_center_z})"
        ),
    )

    with ctx.pose({focus_joint: pi / 2.0, diopter_joint: pi / 3.0}):
        ctx.expect_contact(
            focus_wheel,
            bridge,
            contact_tol=0.001,
            name="focus wheel stays seated when rotated",
        )
        ctx.expect_overlap(
            diopter_ring,
            right_body,
            axes="xz",
            elem_b="eyepiece_barrel",
            min_overlap=0.020,
            name="diopter ring stays coaxial when rotated",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
