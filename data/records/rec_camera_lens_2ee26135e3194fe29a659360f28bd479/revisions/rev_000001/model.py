from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="super_telephoto_lens")

    lens_white = model.material("lens_white", rgba=(0.93, 0.93, 0.90, 1.0))
    matte_black = model.material("matte_black", rgba=(0.08, 0.08, 0.08, 1.0))
    rubber_black = model.material("rubber_black", rgba=(0.13, 0.13, 0.13, 1.0))
    tripod_gray = model.material("tripod_gray", rgba=(0.58, 0.60, 0.63, 1.0))
    metal_dark = model.material("metal_dark", rgba=(0.24, 0.25, 0.28, 1.0))

    def along_x(x: float, y: float = 0.0, z: float = 0.0) -> Origin:
        return Origin(xyz=(x, y, z), rpy=(0.0, math.pi / 2.0, 0.0))

    def along_y(x: float, y: float = 0.0, z: float = 0.0) -> Origin:
        return Origin(xyz=(x, y, z), rpy=(-math.pi / 2.0, 0.0, 0.0))

    def shell_mesh(
        *,
        name: str,
        outer_profile: list[tuple[float, float]],
        inner_profile: list[tuple[float, float]],
        segments: int = 64,
    ):
        return mesh_from_geometry(
            LatheGeometry.from_shell_profiles(
                outer_profile,
                inner_profile,
                segments=segments,
            ),
            name,
        )

    body = model.part("body")
    body.visual(
        Cylinder(radius=0.043, length=0.016),
        origin=along_x(0.008),
        material=metal_dark,
        name="mount_flange",
    )
    body.visual(
        Cylinder(radius=0.050, length=0.046),
        origin=along_x(0.037),
        material=matte_black,
        name="mount_tube",
    )
    body.visual(
        Cylinder(radius=0.068, length=0.162),
        origin=along_x(0.139),
        material=lens_white,
        name="rear_barrel",
    )
    body.visual(
        Cylinder(radius=0.0755, length=0.262),
        origin=along_x(0.349),
        material=lens_white,
        name="body_core",
    )
    body.visual(
        Cylinder(radius=0.0755, length=0.142),
        origin=along_x(0.549),
        material=lens_white,
        name="front_tube",
    )
    body.visual(
        shell_mesh(
            name="hood_shell",
            outer_profile=[(0.086, -0.08), (0.087, -0.02), (0.090, 0.08)],
            inner_profile=[(0.078, -0.08), (0.079, -0.02), (0.082, 0.08)],
        ),
        origin=along_x(0.660),
        material=matte_black,
        name="hood_shell",
    )
    body.visual(
        Cylinder(radius=0.064, length=0.014),
        origin=along_x(0.594),
        material=matte_black,
        name="front_bezel",
    )
    body.visual(
        Cylinder(radius=0.081, length=0.022),
        origin=along_x(0.589),
        material=matte_black,
        name="hood_mount",
    )
    body.visual(
        Cylinder(radius=0.056, length=0.100),
        origin=along_x(0.651),
        material=matte_black,
        name="front_group",
    )

    focus_ring = model.part("focus_ring")
    focus_ring.visual(
        shell_mesh(
            name="focus_shell",
            outer_profile=[
                (0.083, -0.045),
                (0.086, -0.028),
                (0.086, 0.028),
                (0.083, 0.045),
            ],
            inner_profile=[
                (0.0755, -0.045),
                (0.0755, 0.045),
            ],
        ),
        origin=along_x(0.0),
        material=rubber_black,
        name="focus_shell",
    )

    collar = model.part("collar")
    collar.visual(
        shell_mesh(
            name="collar_shell",
            outer_profile=[
                (0.084, -0.045),
                (0.086, -0.020),
                (0.086, 0.020),
                (0.084, 0.045),
            ],
            inner_profile=[
                (0.0755, -0.045),
                (0.0755, 0.045),
            ],
        ),
        origin=along_x(0.0),
        material=tripod_gray,
        name="collar_shell",
    )
    collar.visual(
        Box((0.110, 0.052, 0.042)),
        origin=Origin(xyz=(0.0, 0.0, -0.103)),
        material=tripod_gray,
        name="clamp_block",
    )
    collar.visual(
        Box((0.090, 0.034, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, -0.127)),
        material=tripod_gray,
        name="foot_riser",
    )
    collar.visual(
        Box((0.150, 0.066, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, -0.142)),
        material=tripod_gray,
        name="foot_plate",
    )
    collar.visual(
        Cylinder(radius=0.008, length=0.018),
        origin=along_y(0.032, 0.034, -0.100),
        material=metal_dark,
        name="lever_boss",
    )

    lever = model.part("lever")
    lever.visual(
        Cylinder(radius=0.006, length=0.014),
        origin=along_y(0.0, 0.007, 0.0),
        material=metal_dark,
        name="lever_hub",
    )
    lever.visual(
        Box((0.022, 0.014, 0.014)),
        origin=Origin(xyz=(-0.015, 0.010, -0.002)),
        material=metal_dark,
        name="lever_cam",
    )
    lever.visual(
        Box((0.070, 0.014, 0.010)),
        origin=Origin(xyz=(-0.055, 0.010, -0.010)),
        material=metal_dark,
        name="lever_handle",
    )
    lever.visual(
        Cylinder(radius=0.007, length=0.014),
        origin=along_y(-0.091, 0.010, -0.010),
        material=metal_dark,
        name="lever_tip",
    )

    model.articulation(
        "body_to_focus_ring",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=focus_ring,
        origin=Origin(xyz=(0.525, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=8.0),
    )
    model.articulation(
        "body_to_collar",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=collar,
        origin=Origin(xyz=(0.280, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=4.0),
    )
    model.articulation(
        "collar_to_lever",
        ArticulationType.REVOLUTE,
        parent=collar,
        child=lever,
        origin=Origin(xyz=(0.032, 0.043, -0.100)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.5,
            velocity=2.5,
            lower=0.0,
            upper=1.15,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    focus_ring = object_model.get_part("focus_ring")
    collar = object_model.get_part("collar")
    lever = object_model.get_part("lever")

    focus_joint = object_model.get_articulation("body_to_focus_ring")
    collar_joint = object_model.get_articulation("body_to_collar")
    lever_joint = object_model.get_articulation("collar_to_lever")

    ctx.allow_overlap(
        body,
        focus_ring,
        elem_a="front_tube",
        elem_b="focus_shell",
        reason="The focus ring is a coaxial sleeve riding on the front tube bearing surface.",
    )
    ctx.allow_overlap(
        body,
        collar,
        elem_a="body_core",
        elem_b="collar_shell",
        reason="The tripod collar is modeled as a rotating sleeve clamped around the barrel core.",
    )

    ctx.expect_within(
        body,
        focus_ring,
        axes="yz",
        inner_elem="front_tube",
        outer_elem="focus_shell",
        margin=0.005,
        name="focus ring stays concentric around the front tube",
    )
    ctx.expect_overlap(
        focus_ring,
        body,
        axes="x",
        elem_a="focus_shell",
        elem_b="front_tube",
        min_overlap=0.085,
        name="focus ring covers a substantial section of the front tube",
    )

    ctx.expect_within(
        body,
        collar,
        axes="yz",
        inner_elem="body_core",
        outer_elem="collar_shell",
        margin=0.003,
        name="tripod collar stays concentric around the barrel core",
    )
    ctx.expect_overlap(
        collar,
        body,
        axes="x",
        elem_a="collar_shell",
        elem_b="body_core",
        min_overlap=0.085,
        name="tripod collar wraps a real section of barrel",
    )
    ctx.expect_contact(
        lever,
        collar,
        elem_a="lever_hub",
        elem_b="lever_boss",
        name="locking lever stays mounted on the collar boss",
    )

    with ctx.pose(
        {
            focus_joint: 1.4,
            collar_joint: math.pi / 2.0,
        }
    ):
        ctx.expect_within(
            body,
            focus_ring,
            axes="yz",
            inner_elem="front_tube",
            outer_elem="focus_shell",
            margin=0.005,
            name="focus ring remains concentric while rotated",
        )
        ctx.expect_within(
            body,
            collar,
            axes="yz",
            inner_elem="body_core",
            outer_elem="collar_shell",
            margin=0.003,
            name="tripod collar remains concentric while rotated",
        )

    closed_tip = ctx.part_element_world_aabb(lever, elem="lever_tip")
    open_tip = None
    with ctx.pose({lever_joint: 0.95}):
        ctx.expect_contact(
            lever,
            collar,
            elem_a="lever_hub",
            elem_b="lever_boss",
            name="locking lever keeps its pivot contact when opened",
        )
        open_tip = ctx.part_element_world_aabb(lever, elem="lever_tip")

    closed_tip_z = closed_tip[1][2] if closed_tip is not None else None
    open_tip_z = open_tip[1][2] if open_tip is not None else None
    ctx.check(
        "locking lever lifts upward when opened",
        closed_tip_z is not None and open_tip_z is not None and open_tip_z > closed_tip_z + 0.035,
        details=f"closed_tip_z={closed_tip_z}, open_tip_z={open_tip_z}",
    )

    return ctx.report()


object_model = build_object_model()
