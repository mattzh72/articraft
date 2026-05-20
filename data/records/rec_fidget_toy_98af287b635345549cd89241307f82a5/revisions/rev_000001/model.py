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
    Inertial,
    LatheGeometry,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _save_mesh(name: str, geometry: MeshGeometry):
    return mesh_from_geometry(geometry, name)


def _build_head_shell() -> MeshGeometry:
    outer_profile = [
        (0.0128, -0.0036),
        (0.0132, -0.0028),
        (0.0132, 0.0028),
        (0.0128, 0.0036),
    ]
    inner_profile = [
        (0.0096, -0.0021),
        (0.0096, 0.0021),
    ]
    return LatheGeometry.from_shell_profiles(
        outer_profile,
        inner_profile,
        segments=64,
        start_cap="flat",
        end_cap="flat",
    )


def _build_ratchet_gear() -> MeshGeometry:
    gear = CylinderGeometry(radius=0.0082, height=0.0034, radial_segments=40)
    root_band = CylinderGeometry(radius=0.0088, height=0.0034, radial_segments=40)
    gear.merge(root_band)

    tooth_count = 20
    tooth_radius = 0.0093
    for tooth_index in range(tooth_count):
        angle = (2.0 * math.pi * tooth_index) / tooth_count
        tooth = BoxGeometry((0.0017, 0.0013, 0.0034))
        tooth.translate(tooth_radius, 0.0, 0.0).rotate_z(angle)
        gear.merge(tooth)

    hub = CylinderGeometry(radius=0.0038, height=0.0036, radial_segments=24)
    hub.translate(0.0, 0.0, 0.0001)
    gear.merge(hub)
    return gear


def _build_pawl() -> MeshGeometry:
    pawl = CylinderGeometry(radius=0.00145, height=0.0012, radial_segments=20)
    arm = BoxGeometry((0.0054, 0.0018, 0.0012))
    arm.translate(0.0018, 0.0, 0.0)
    tip = BoxGeometry((0.0017, 0.0020, 0.0012))
    tip.translate(0.0039, -0.00125, 0.0)
    tail = BoxGeometry((0.0019, 0.0017, 0.0012))
    tail.translate(-0.0011, 0.00035, 0.0)
    pawl.merge(arm)
    pawl.merge(tip)
    pawl.merge(tail)
    return pawl


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="mini_ratchet_wrench_fidget")

    satin_steel = model.material("satin_steel", rgba=(0.74, 0.76, 0.79, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.29, 0.31, 0.34, 1.0))
    gunmetal = model.material("gunmetal", rgba=(0.38, 0.40, 0.44, 1.0))

    body = model.part("body")
    body.visual(
        Box((0.064, 0.012, 0.0034)),
        origin=Origin(xyz=(-0.042, 0.0, 0.0)),
        material=satin_steel,
        name="handle_bar",
    )
    body.visual(
        Box((0.012, 0.015, 0.0042)),
        origin=Origin(xyz=(-0.012, 0.0, 0.0)),
        material=satin_steel,
        name="neck_web",
    )
    body.visual(
        _save_mesh("head_shell", _build_head_shell()),
        material=gunmetal,
        name="head_shell",
    )
    body.visual(
        Cylinder(radius=0.0016, length=0.0014),
        origin=Origin(xyz=(-0.0044, 0.0112, 0.0039)),
        material=dark_steel,
        name="pivot_head",
    )
    body.inertial = Inertial.from_geometry(
        Box((0.090, 0.028, 0.012)),
        mass=0.09,
        origin=Origin(xyz=(-0.019, 0.0, 0.0)),
    )

    ratchet = model.part("ratchet")
    ratchet.visual(
        _save_mesh("ratchet_gear", _build_ratchet_gear()),
        material=dark_steel,
        name="gear_body",
    )
    ratchet.visual(
        Box((0.0038, 0.0038, 0.0023)),
        origin=Origin(xyz=(0.0, 0.0, 0.00275)),
        material=satin_steel,
        name="drive_boss",
    )
    ratchet.inertial = Inertial.from_geometry(
        Cylinder(radius=0.0095, length=0.0057),
        mass=0.02,
        origin=Origin(xyz=(0.0, 0.0, 0.0008)),
    )

    pawl = model.part("pawl")
    pawl.visual(
        _save_mesh("pawl_arm", _build_pawl()),
        material=satin_steel,
        name="pawl_arm",
    )
    pawl.inertial = Inertial.from_geometry(
        Box((0.0080, 0.0040, 0.0012)),
        mass=0.003,
        origin=Origin(xyz=(0.0011, -0.0001, 0.0)),
    )

    model.articulation(
        "body_to_ratchet",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=ratchet,
        origin=Origin(),
        # Positive spin follows the visible advance direction when viewed from
        # the front face of the wrench head.
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.6, velocity=18.0),
    )
    model.articulation(
        "body_to_pawl",
        ArticulationType.REVOLUTE,
        parent=body,
        child=pawl,
        origin=Origin(xyz=(-0.0044, 0.0112, 0.0), rpy=(0.0, 0.0, 0.38)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.08, velocity=4.0, lower=0.0, upper=0.22),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    ratchet = object_model.get_part("ratchet")
    head_spin = object_model.get_articulation("body_to_ratchet")
    pawl = object_model.get_part("pawl")
    pawl_joint = object_model.get_articulation("body_to_pawl")

    ctx.expect_overlap(
        ratchet,
        body,
        axes="xy",
        elem_a="gear_body",
        elem_b="head_shell",
        min_overlap=0.016,
        name="ratchet stays nested in head housing footprint",
    )
    with ctx.pose({head_spin: math.pi / 2.0}):
        ctx.expect_overlap(
            ratchet,
            body,
            axes="xy",
            elem_a="gear_body",
            elem_b="head_shell",
            min_overlap=0.016,
            name="ratchet remains centered while spinning",
        )
    ctx.expect_overlap(
        pawl,
        body,
        axes="xy",
        elem_a="pawl_arm",
        elem_b="head_shell",
        min_overlap=0.003,
        name="pawl stays inside head housing footprint",
    )
    rest_aabb = ctx.part_element_world_aabb(pawl, elem="pawl_arm")
    with ctx.pose({pawl_joint: 0.20}):
        ctx.expect_overlap(
            pawl,
            body,
            axes="xy",
            elem_a="pawl_arm",
            elem_b="head_shell",
            min_overlap=0.002,
            name="pawl remains housed through its swing",
        )
        swung_aabb = ctx.part_element_world_aabb(pawl, elem="pawl_arm")
    rest_center_y = None if rest_aabb is None else (rest_aabb[0][1] + rest_aabb[1][1]) * 0.5
    swung_center_y = None if swung_aabb is None else (swung_aabb[0][1] + swung_aabb[1][1]) * 0.5
    ctx.check(
        "positive pawl motion retracts the tip",
        rest_center_y is not None
        and swung_center_y is not None
        and swung_center_y > rest_center_y + 0.0002,
        details=f"rest_center_y={rest_center_y}, swung_center_y={swung_center_y}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
