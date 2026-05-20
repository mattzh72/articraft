from __future__ import annotations

import math

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
    mesh_from_cadquery,
)
import cadquery as cq


SHOULDER_Z = 0.92
FIRST_LINK_LENGTH = 1.02
SECOND_LINK_LENGTH = 0.78
WRIST_STANDOFF = 0.05


def _cyl_z(radius: float, length: float, center: tuple[float, float, float]):
    """CadQuery cylinder with its axis along +Z and centered at center."""
    return (
        cq.Workplane("XY")
        .circle(radius)
        .extrude(length)
        .translate((center[0], center[1], center[2] - length * 0.5))
    )


def _cyl_y(radius: float, length: float, center: tuple[float, float, float]):
    return _cyl_z(radius, length, (0.0, 0.0, 0.0)).rotate(
        (0, 0, 0), (1, 0, 0), -90
    ).translate(center)


def _cyl_x(radius: float, length: float, center: tuple[float, float, float]):
    return _cyl_z(radius, length, (0.0, 0.0, 0.0)).rotate(
        (0, 0, 0), (0, 1, 0), 90
    ).translate(center)


def _box(size: tuple[float, float, float], center: tuple[float, float, float]):
    return cq.Workplane("XY").box(*size).translate(center)


def _union(shapes):
    result = shapes[0]
    for shape in shapes[1:]:
        result = result.union(shape)
    return result


def _base_casting():
    """One connected casting for the fixed pedestal and shoulder fork."""
    shapes = [
        _cyl_z(0.34, 0.10, (0.0, 0.0, 0.05)),
        _cyl_z(0.25, 0.055, (0.0, 0.0, 0.125)),
        _cyl_z(0.16, 0.44, (0.0, 0.0, 0.36)),
        _box((0.40, 0.52, 0.14), (0.0, 0.0, 0.61)),
        _box((0.32, 0.075, 0.66), (0.0, 0.242, 0.91)),
        _box((0.32, 0.075, 0.66), (0.0, -0.242, 0.91)),
        # rear bridge and lower saddle make the fork unambiguously fixed
        _box((0.11, 0.43, 0.61), (-0.265, 0.0, 0.905)),
        _box((0.11, 0.43, 0.11), (-0.175, 0.0, 0.645)),
        _box((0.28, 0.45, 0.10), (0.0, 0.0, 0.64)),
        # raised bearing bosses and machined outer flanges on the cheeks
        _cyl_y(0.205, 0.036, (0.0, 0.292, SHOULDER_Z)),
        _cyl_y(0.205, 0.036, (0.0, -0.292, SHOULDER_Z)),
        _cyl_y(0.145, 0.050, (0.0, 0.315, SHOULDER_Z)),
        _cyl_y(0.145, 0.050, (0.0, -0.315, SHOULDER_Z)),
        # hard stops well outside the swept first-link envelope
        _box((0.050, 0.38, 0.13), (-0.240, 0.0, 1.145)),
        _box((0.050, 0.38, 0.13), (-0.240, 0.0, 0.685)),
    ]
    return _union(shapes)


def _first_link_casting():
    """Deep first moving link with shoulder trunnion and distal clevis."""
    shapes = [
        _cyl_y(0.162, 0.315, (0.0, 0.0, 0.0)),
        _box((0.72, 0.165, 0.285), (0.43, 0.0, 0.0)),
        _box((0.22, 0.19, 0.24), (0.14, 0.0, 0.0)),
        # distal clevis: two outboard cheeks plus a rear bridge, leaving a clear gap
        _box((0.285, 0.062, 0.275), (0.97, 0.142, 0.0)),
        _box((0.285, 0.062, 0.275), (0.97, -0.142, 0.0)),
        _box((0.105, 0.335, 0.205), (0.815, 0.0, 0.0)),
        _cyl_y(0.135, 0.065, (1.02, 0.178, 0.0)),
        _cyl_y(0.135, 0.065, (1.02, -0.178, 0.0)),
        # shallow raised inspection pads on both side faces
        _box((0.34, 0.014, 0.155), (0.48, 0.086, 0.0)),
        _box((0.34, 0.014, 0.155), (0.48, -0.086, 0.0)),
        # top rib and underside rib to make the section read as a deep casting
        _box((0.54, 0.075, 0.045), (0.48, 0.0, 0.162)),
        _box((0.48, 0.065, 0.035), (0.50, 0.0, -0.160)),
    ]
    return _union(shapes)


def _second_link_casting():
    """Slimmer second moving link nested between the first link's elbow cheeks."""
    shapes = [
        _cyl_y(0.112, 0.180, (0.0, 0.0, 0.0)),
        _box((0.545, 0.120, 0.175), (0.34, 0.0, 0.0)),
        _box((0.18, 0.145, 0.155), (0.12, 0.0, 0.0)),
        _box((0.135, 0.135, 0.205), (0.675, 0.0, 0.0)),
        _cyl_x(0.132, 0.100, (0.780, 0.0, 0.0)),
        _cyl_x(0.102, 0.070, (0.795, 0.0, 0.0)),
        # small flush inspection covers
        _box((0.255, 0.012, 0.103), (0.39, 0.063, 0.0)),
        _box((0.255, 0.012, 0.103), (0.39, -0.063, 0.0)),
        _box((0.38, 0.052, 0.030), (0.39, 0.0, 0.102)),
    ]
    return _union(shapes)


def _wrist_nose_casting():
    """Compact cylindrical wrist with a flange and an off-axis timing lug."""
    shapes = [
        _cyl_x(0.092, 0.245, (0.145, 0.0, 0.0)),
        _cyl_x(0.125, 0.042, (0.021, 0.0, 0.0)),
        _cyl_x(0.118, 0.045, (0.275, 0.0, 0.0)),
        _cyl_x(0.072, 0.042, (0.310, 0.0, 0.0)),
        _box((0.070, 0.036, 0.032), (0.165, 0.0, 0.096)),
        _box((0.060, 0.030, 0.026), (0.285, 0.0, 0.098)),
    ]
    # four proud bolt heads on the tool flange, embedded into the face
    for angle in (math.pi / 4.0, 3.0 * math.pi / 4.0, 5.0 * math.pi / 4.0, 7.0 * math.pi / 4.0):
        shapes.append(
            _cyl_x(
                0.013,
                0.014,
                (0.304, 0.070 * math.cos(angle), 0.070 * math.sin(angle)),
            )
        )
    return _union(shapes)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="machine_tending_arm")

    cast_steel = model.material("cast_steel", rgba=(0.30, 0.34, 0.37, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.10, 0.11, 0.12, 1.0))
    bare_steel = model.material("bare_steel", rgba=(0.63, 0.65, 0.66, 1.0))
    orange = model.material("safety_orange", rgba=(0.95, 0.36, 0.08, 1.0))
    link_gray = model.material("warm_link_gray", rgba=(0.70, 0.72, 0.70, 1.0))
    cover_black = model.material("cover_black", rgba=(0.04, 0.045, 0.05, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_base_casting(), "pedestal_fork_casting"),
        origin=Origin(),
        material=cast_steel,
        name="pedestal_fork_casting",
    )
    base.visual(
        Cylinder(radius=0.36, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.158)),
        material=bare_steel,
        name="pedestal_flange_ring",
    )
    # Four embedded anchor pads distinguish the fixed support from moving links.
    for index, (x, y) in enumerate(((0.23, 0.23), (-0.23, 0.23), (-0.23, -0.23), (0.23, -0.23))):
        base.visual(
            Cylinder(radius=0.027, length=0.014),
            origin=Origin(xyz=(x, y, 0.105)),
            material=dark_steel,
            name=f"anchor_bolt_{index}",
        )
    base.inertial = Inertial.from_geometry(
        Cylinder(radius=0.34, length=1.18),
        mass=185.0,
        origin=Origin(xyz=(0.0, 0.0, 0.60)),
    )

    first_link = model.part("first_link")
    first_link.visual(
        mesh_from_cadquery(_first_link_casting(), "deep_link_casting"),
        origin=Origin(),
        material=orange,
        name="deep_link_casting",
    )
    first_link.visual(
        Cylinder(radius=0.108, length=0.410),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=bare_steel,
        name="shoulder_trunnion_sleeve",
    )
    first_link.inertial = Inertial.from_geometry(
        Box((1.12, 0.34, 0.32)),
        mass=62.0,
        origin=Origin(xyz=(0.54, 0.0, 0.0)),
    )

    second_link = model.part("second_link")
    second_link.visual(
        mesh_from_cadquery(_second_link_casting(), "slim_link_casting"),
        origin=Origin(),
        material=link_gray,
        name="slim_link_casting",
    )
    second_link.visual(
        Cylinder(radius=0.080, length=0.222),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=bare_steel,
        name="elbow_trunnion_sleeve",
    )
    second_link.visual(
        Box((0.220, 0.010, 0.082)),
        origin=Origin(xyz=(0.39, 0.064, 0.0)),
        material=cover_black,
        name="side_cover_0",
    )
    second_link.visual(
        Box((0.220, 0.010, 0.082)),
        origin=Origin(xyz=(0.39, -0.064, 0.0)),
        material=cover_black,
        name="side_cover_1",
    )
    second_link.inertial = Inertial.from_geometry(
        Box((0.86, 0.18, 0.23)),
        mass=30.0,
        origin=Origin(xyz=(0.38, 0.0, 0.0)),
    )

    wrist_nose = model.part("wrist_nose")
    wrist_nose.visual(
        mesh_from_cadquery(_wrist_nose_casting(), "wrist_nose_casting"),
        origin=Origin(),
        material=dark_steel,
        name="wrist_nose_casting",
    )
    wrist_nose.visual(
        Cylinder(radius=0.062, length=0.052),
        origin=Origin(xyz=(0.327, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=bare_steel,
        name="tool_register",
    )
    wrist_nose.inertial = Inertial.from_geometry(
        Cylinder(radius=0.13, length=0.35),
        mass=12.0,
        origin=Origin(xyz=(0.16, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    model.articulation(
        "shoulder_pitch",
        ArticulationType.REVOLUTE,
        parent=base,
        child=first_link,
        origin=Origin(xyz=(0.0, 0.0, SHOULDER_Z)),
        # The link geometry extends along local +X; -Y makes positive q lift it.
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=1250.0, velocity=1.1, lower=-0.15, upper=1.10),
    )
    model.articulation(
        "elbow_pitch",
        ArticulationType.REVOLUTE,
        parent=first_link,
        child=second_link,
        origin=Origin(xyz=(FIRST_LINK_LENGTH, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=780.0, velocity=1.35, lower=-0.15, upper=1.45),
    )
    model.articulation(
        "wrist_roll",
        ArticulationType.REVOLUTE,
        parent=second_link,
        child=wrist_nose,
        origin=Origin(xyz=(SECOND_LINK_LENGTH + WRIST_STANDOFF, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=240.0,
            velocity=4.0,
            lower=-math.pi,
            upper=math.pi,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    first_link = object_model.get_part("first_link")
    second_link = object_model.get_part("second_link")
    wrist_nose = object_model.get_part("wrist_nose")
    shoulder = object_model.get_articulation("shoulder_pitch")
    elbow = object_model.get_articulation("elbow_pitch")
    wrist = object_model.get_articulation("wrist_roll")

    ctx.allow_overlap(
        base,
        first_link,
        elem_a="pedestal_fork_casting",
        elem_b="shoulder_trunnion_sleeve",
        reason=(
            "The shoulder sleeve is intentionally modeled as a captured shaft "
            "seated in the fork bearing bores."
        ),
    )
    ctx.allow_overlap(
        first_link,
        second_link,
        elem_a="deep_link_casting",
        elem_b="elbow_trunnion_sleeve",
        reason=(
            "The elbow sleeve is intentionally modeled as the bearing shaft "
            "passing through the first-link clevis cheeks."
        ),
    )

    ctx.expect_overlap(
        first_link,
        base,
        axes="xz",
        elem_a="shoulder_trunnion_sleeve",
        elem_b="pedestal_fork_casting",
        min_overlap=0.10,
        name="shoulder trunnion sits inside fork envelope",
    )
    ctx.expect_within(
        first_link,
        base,
        axes="y",
        inner_elem="shoulder_trunnion_sleeve",
        outer_elem="pedestal_fork_casting",
        margin=0.002,
        name="shoulder sleeve is retained across fork width",
    )
    ctx.expect_within(
        second_link,
        first_link,
        axes="y",
        inner_elem="elbow_trunnion_sleeve",
        outer_elem="deep_link_casting",
        margin=0.010,
        name="elbow trunnion is captured between first-link cheeks",
    )
    ctx.expect_origin_gap(
        second_link,
        first_link,
        axis="x",
        min_gap=0.95,
        max_gap=1.09,
        name="elbow root is at distal first link",
    )
    ctx.expect_origin_gap(
        wrist_nose,
        second_link,
        axis="x",
        min_gap=0.80,
        max_gap=0.86,
        name="wrist root is at distal second link",
    )

    rest_elbow = ctx.part_world_position(second_link)
    rest_wrist = ctx.part_world_position(wrist_nose)
    with ctx.pose({shoulder: 1.0}):
        raised_elbow = ctx.part_world_position(second_link)
    with ctx.pose({elbow: 1.05}):
        raised_wrist = ctx.part_world_position(wrist_nose)
    with ctx.pose({shoulder: -0.15, elbow: -0.15}):
        low_aabb = ctx.part_world_aabb(wrist_nose)

    ctx.check(
        "shoulder pitch raises elbow",
        rest_elbow is not None
        and raised_elbow is not None
        and raised_elbow[2] > rest_elbow[2] + 0.50,
        details=f"rest={rest_elbow}, raised={raised_elbow}",
    )
    ctx.check(
        "elbow pitch raises wrist",
        rest_wrist is not None
        and raised_wrist is not None
        and raised_wrist[2] > rest_wrist[2] + 0.45,
        details=f"rest={rest_wrist}, raised={raised_wrist}",
    )
    ctx.check(
        "low service pose clears pedestal",
        low_aabb is not None and low_aabb[0][2] > 0.20,
        details=f"wrist_aabb={low_aabb}",
    )
    ctx.check(
        "parallel shoulder and elbow axes",
        tuple(shoulder.axis) == tuple(elbow.axis) == (0.0, -1.0, 0.0)
        and tuple(wrist.axis) == (1.0, 0.0, 0.0),
        details=f"shoulder={shoulder.axis}, elbow={elbow.axis}, wrist={wrist.axis}",
    )

    return ctx.report()


object_model = build_object_model()
