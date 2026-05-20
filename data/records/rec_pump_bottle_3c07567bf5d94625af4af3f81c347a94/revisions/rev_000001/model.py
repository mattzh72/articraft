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
    ExtrudeWithHolesGeometry,
    Inertial,
    LatheGeometry,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _circle_profile(radius: float, *, segments: int = 36) -> list[tuple[float, float]]:
    return [
        (
            radius * math.cos((2.0 * math.pi * index) / segments),
            radius * math.sin((2.0 * math.pi * index) / segments),
        )
        for index in range(segments)
    ]


def _ring_mesh(
    *,
    outer_radius: float,
    inner_radius: float,
    height: float,
    name: str,
    segments: int = 40,
):
    return mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            _circle_profile(outer_radius, segments=segments),
            [_circle_profile(inner_radius, segments=segments)],
            height,
            center=False,
        ),
        name,
    )


def _build_collar_mesh():
    segments = 56
    geometry = MeshGeometry()
    geometry.merge(
        ExtrudeWithHolesGeometry(
            _circle_profile(0.0148, segments=segments),
            [_circle_profile(0.0098, segments=segments)],
            0.0040,
            center=False,
        )
    )
    geometry.merge(
        ExtrudeWithHolesGeometry(
            _circle_profile(0.0136, segments=segments),
            [_circle_profile(0.0096, segments=segments)],
            0.0030,
            center=False,
        ).translate(0.0, 0.0, 0.0040)
    )
    geometry.merge(BoxGeometry((0.0060, 0.0050, 0.0100)).translate(0.0092, 0.0108, 0.0050))
    geometry.merge(
        CylinderGeometry(radius=0.0014, height=0.0020, radial_segments=18).translate(
            0.0092,
            0.0108,
            0.0080,
        )
    )
    geometry.merge(
        CylinderGeometry(radius=0.0014, height=0.0020, radial_segments=18).translate(
            0.0092,
            0.0108,
            0.0120,
        )
    )
    return mesh_from_geometry(geometry, "pump_collar_shell")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="travel_pump_bottle")

    bottle_plastic = model.material("bottle_plastic", rgba=(0.90, 0.88, 0.83, 1.0))
    pump_white = model.material("pump_white", rgba=(0.93, 0.94, 0.95, 1.0))
    nozzle_white = model.material("nozzle_white", rgba=(0.96, 0.97, 0.98, 1.0))
    clear_cap = model.material("clear_cap", rgba=(0.80, 0.88, 0.96, 0.48))

    body = model.part("body")
    body.visual(
        mesh_from_geometry(
            LatheGeometry.from_shell_profiles(
                [
                    (0.0202, 0.0000),
                    (0.0226, 0.0035),
                    (0.0229, 0.0110),
                    (0.0229, 0.0640),
                    (0.0214, 0.0715),
                    (0.0180, 0.0770),
                    (0.0140, 0.0810),
                    (0.0110, 0.0820),
                    (0.0110, 0.0920),
                ],
                [
                    (0.0000, 0.0024),
                    (0.0193, 0.0045),
                    (0.0197, 0.0110),
                    (0.0197, 0.0640),
                    (0.0183, 0.0710),
                    (0.0150, 0.0760),
                    (0.0118, 0.0805),
                    (0.0094, 0.0820),
                    (0.0094, 0.0920),
                ],
                segments=64,
            ),
            "body_shell",
        ),
        material=bottle_plastic,
        name="body_shell",
    )
    body.inertial = Inertial.from_geometry(
        Cylinder(radius=0.023, length=0.094),
        mass=0.14,
        origin=Origin(xyz=(0.0, 0.0, 0.046)),
    )

    collar = model.part("pump_collar")
    collar.visual(
        _build_collar_mesh(),
        material=pump_white,
        name="collar_shell",
    )
    collar.inertial = Inertial.from_geometry(
        Cylinder(radius=0.018, length=0.018),
        mass=0.03,
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
    )

    plunger = model.part("plunger")
    plunger.visual(
        Cylinder(radius=0.0046, length=0.0120),
        origin=Origin(xyz=(0.0, 0.0, 0.0030)),
        material=nozzle_white,
        name="guide_plug",
    )
    plunger.visual(
        Cylinder(radius=0.0115, length=0.0040),
        origin=Origin(xyz=(0.0, 0.0, 0.0050)),
        material=nozzle_white,
        name="button_disk",
    )
    plunger.visual(
        Cylinder(radius=0.0102, length=0.0016),
        origin=Origin(xyz=(0.0, 0.0, 0.0082)),
        material=nozzle_white,
        name="button_top",
    )
    plunger.visual(
        Cylinder(radius=0.0026, length=0.0100),
        origin=Origin(xyz=(0.0055, 0.0, 0.0072), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=nozzle_white,
        name="nozzle_barrel",
    )
    plunger.visual(
        Cylinder(radius=0.0022, length=0.0048),
        origin=Origin(xyz=(0.0115, 0.0, 0.0072), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=nozzle_white,
        name="nozzle_tip",
    )
    plunger.inertial = Inertial.from_geometry(
        Cylinder(radius=0.012, length=0.022),
        mass=0.025,
        origin=Origin(xyz=(0.003, 0.0, 0.008)),
    )

    cap = model.part("flip_cap")
    cap.visual(
        Cylinder(radius=0.0018, length=0.0020),
        material=clear_cap,
        name="hinge_knuckle",
    )
    cap.visual(
        Box((0.0030, 0.0030, 0.0060)),
        origin=Origin(xyz=(0.0015, -0.0008, 0.0010)),
        material=clear_cap,
        name="cap_web",
    )
    cap.visual(
        Box((0.0130, 0.0095, 0.0016)),
        origin=Origin(xyz=(0.0074, -0.0060, 0.0056)),
        material=clear_cap,
        name="cap_top",
    )
    cap.visual(
        Box((0.0130, 0.0016, 0.0080)),
        origin=Origin(xyz=(0.0074, -0.0102, 0.0024)),
        material=clear_cap,
        name="cap_side_outer",
    )
    cap.visual(
        Box((0.0130, 0.0016, 0.0080)),
        origin=Origin(xyz=(0.0074, -0.0026, 0.0024)),
        material=clear_cap,
        name="cap_side_inner",
    )
    cap.visual(
        Box((0.0016, 0.0095, 0.0080)),
        origin=Origin(xyz=(0.0145, -0.0060, 0.0024)),
        material=clear_cap,
        name="cap_front_wall",
    )
    cap.inertial = Inertial.from_geometry(
        Box((0.016, 0.011, 0.009)),
        mass=0.01,
        origin=Origin(xyz=(0.007, -0.006, 0.003)),
    )

    model.articulation(
        "body_to_collar",
        ArticulationType.FIXED,
        parent=body,
        child=collar,
        origin=Origin(xyz=(0.0, 0.0, 0.0920)),
    )
    model.articulation(
        "collar_to_plunger",
        ArticulationType.PRISMATIC,
        parent=collar,
        child=plunger,
        origin=Origin(xyz=(0.0, 0.0, 0.0100)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=0.05,
            lower=0.0,
            upper=0.0035,
        ),
    )
    model.articulation(
        "collar_to_cap",
        ArticulationType.REVOLUTE,
        parent=collar,
        child=cap,
        origin=Origin(xyz=(0.0092, 0.0108, 0.0105)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=4.0,
            lower=0.0,
            upper=2.1,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    collar = object_model.get_part("pump_collar")
    plunger = object_model.get_part("plunger")
    cap = object_model.get_part("flip_cap")

    plunger_slide = object_model.get_articulation("collar_to_plunger")
    cap_hinge = object_model.get_articulation("collar_to_cap")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.allow_overlap(
        body,
        collar,
        reason="Simplified pump collar nests over the bottle neck without modeled threads.",
    )
    ctx.allow_overlap(
        plunger,
        collar,
        reason="Simplified internal pump stem runs within the collar guide bore.",
    )
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_overlap(collar, body, axes="xy", min_overlap=0.020, name="collar centered on bottle neck")
    ctx.expect_gap(collar, body, axis="z", min_gap=-0.0005, max_gap=0.004, name="collar seats at neck top")
    ctx.expect_overlap(plunger, collar, axes="xy", min_overlap=0.009, name="plunger stays centered in collar")

    with ctx.pose({plunger_slide: 0.0, cap_hinge: 0.0}):
        ctx.expect_overlap(
            cap,
            plunger,
            axes="yz",
            min_overlap=0.0015,
            name="closed cap aligns over nozzle",
        )
        ctx.expect_overlap(
            cap,
            plunger,
            axes="y",
            min_overlap=0.001,
            elem_a="cap_front_wall",
            elem_b="nozzle_tip",
            name="cap front wall sits in front of nozzle outlet",
        )
        ctx.expect_gap(
            cap,
            plunger,
            axis="x",
            min_gap=0.0004,
            max_gap=0.0120,
            positive_elem="cap_front_wall",
            negative_elem="nozzle_tip",
            name="cap front wall clears nozzle tip",
        )

    plunger_rest = ctx.part_world_position(plunger)
    assert plunger_rest is not None
    with ctx.pose({plunger_slide: 0.0035}):
        plunger_pressed = ctx.part_world_position(plunger)
        assert plunger_pressed is not None
        assert plunger_pressed[2] < plunger_rest[2] - 0.003
        ctx.expect_overlap(plunger, collar, axes="xy", min_overlap=0.009, name="pressed plunger stays centered")

    cap_rest = ctx.part_world_position(cap)
    assert cap_rest is not None
    cap_rest_aabb = ctx.part_world_aabb(cap)
    assert cap_rest_aabb is not None
    with ctx.pose({cap_hinge: 1.9}):
        cap_open = ctx.part_world_position(cap)
        assert cap_open is not None
        cap_open_aabb = ctx.part_world_aabb(cap)
        assert cap_open_aabb is not None
        assert all(abs(cap_open[i] - cap_rest[i]) < 1e-9 for i in range(3))
        assert cap_open_aabb[0][1] > cap_rest_aabb[0][1] + 0.008
        ctx.expect_gap(
            cap,
            plunger,
            axis="y",
            min_gap=0.004,
            positive_elem="cap_top",
            negative_elem="nozzle_tip",
            name="open cap swings clear beside nozzle",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
