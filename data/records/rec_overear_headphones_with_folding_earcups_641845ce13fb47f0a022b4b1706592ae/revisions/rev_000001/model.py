from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


CUP_HALF_HEIGHT = 0.064
CUP_HALF_WIDTH = 0.047
CUP_THICKNESS = 0.050
YOKE_DROP = 0.132
FOLD_Z = 0.092
SIDE_X = 0.112


def _headband_arch() -> cq.Workplane:
    """A broad, gently arched band extruded through the front/back direction."""
    outer_rx = 0.145
    outer_rz = 0.205
    inner_rx = 0.111
    inner_rz = 0.165
    base_z = 0.112
    n = 26

    outer = []
    for i in range(n + 1):
        x = -outer_rx + (2.0 * outer_rx * i / n)
        z = base_z + outer_rz * math.sqrt(max(0.0, 1.0 - (x / outer_rx) ** 2))
        outer.append((x, z))

    inner = []
    for i in range(n + 1):
        x = inner_rx - (2.0 * inner_rx * i / n)
        z = base_z + inner_rz * math.sqrt(max(0.0, 1.0 - (x / inner_rx) ** 2))
        inner.append((x, z))

    profile = outer + inner
    return cq.Workplane("XZ").polyline(profile).close().extrude(0.013, both=True)


def _headband_pad() -> cq.Workplane:
    """Soft underside pad following the lower edge of the band."""
    outer_rx = 0.116
    outer_rz = 0.160
    inner_rx = 0.088
    inner_rz = 0.134
    base_z = 0.101
    n = 22

    outer = []
    for i in range(n + 1):
        x = -outer_rx + (2.0 * outer_rx * i / n)
        z = base_z + outer_rz * math.sqrt(max(0.0, 1.0 - (x / outer_rx) ** 2))
        outer.append((x, z))

    inner = []
    for i in range(n + 1):
        x = inner_rx - (2.0 * inner_rx * i / n)
        z = base_z + inner_rz * math.sqrt(max(0.0, 1.0 - (x / inner_rx) ** 2))
        inner.append((x, z))

    return cq.Workplane("XZ").polyline(outer + inner).close().extrude(0.010, both=True)


def _oval_disc(y_radius: float, z_radius: float, thickness: float) -> cq.Workplane:
    """Oval extrusion whose thickness is along local X."""
    return cq.Workplane("YZ").ellipse(y_radius, z_radius).extrude(thickness / 2.0, both=True)


def _oval_ring(
    outer_y: float,
    outer_z: float,
    inner_y: float,
    inner_z: float,
    thickness: float,
) -> cq.Workplane:
    """Pillow-like oval cushion ring with a real opening."""
    return (
        cq.Workplane("YZ")
        .ellipse(outer_y, outer_z)
        .ellipse(inner_y, inner_z)
        .extrude(thickness / 2.0, both=True)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="fold_flat_noise_canceling_headphones")

    matte_black = Material("matte_black_plastic", rgba=(0.005, 0.006, 0.007, 1.0))
    satin_black = Material("satin_black_shell", rgba=(0.015, 0.016, 0.018, 1.0))
    soft_pad = Material("soft_charcoal_cushion", rgba=(0.025, 0.023, 0.022, 1.0))
    dark_metal = Material("dark_anodized_metal", rgba=(0.17, 0.17, 0.18, 1.0))
    accent = Material("smoked_accent_plate", rgba=(0.035, 0.038, 0.043, 1.0))

    headband = model.part("headband")
    headband.visual(
        mesh_from_cadquery(_headband_arch(), "headband_arch", tolerance=0.0015),
        material=matte_black,
        name="outer_arch",
    )
    headband.visual(
        mesh_from_cadquery(_headband_pad(), "headband_pad", tolerance=0.0015),
        material=soft_pad,
        name="inner_pad",
    )

    # Forked fold-hinge clevises at the ends of the band.  The central pin spans
    # through the yoke barrel while the side ears leave a visible gap for it.
    for side_name, sx in (("left", -SIDE_X), ("right", SIDE_X)):
        headband.visual(
            Box((0.033, 0.082, 0.018)),
            origin=Origin(xyz=(sx, 0.0, FOLD_Z + 0.024)),
            material=matte_black,
            name=f"{side_name}_hinge_bridge",
        )
        for yy, cue in ((-0.035, "front"), (0.035, "rear")):
            headband.visual(
                Box((0.025, 0.016, 0.038)),
                origin=Origin(xyz=(sx, yy, FOLD_Z + 0.005)),
                material=matte_black,
                name=f"{side_name}_{cue}_hinge_ear",
            )
            headband.visual(
                Cylinder(radius=0.011, length=0.016),
                origin=Origin(xyz=(sx, yy, FOLD_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
                material=dark_metal,
                name=f"{side_name}_{cue}_hinge_knuckle",
            )
        headband.visual(
            Cylinder(radius=0.0045, length=0.096),
            origin=Origin(xyz=(sx, 0.0, FOLD_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=dark_metal,
            name=f"{side_name}_fold_pin",
        )

    for side_name, sx, fold_axis, swivel_axis, cushion_x, cover_x in (
        ("left", -SIDE_X, (0.0, -1.0, 0.0), (0.0, 1.0, 0.0), 0.028, -0.027),
        ("right", SIDE_X, (0.0, 1.0, 0.0), (0.0, -1.0, 0.0), -0.028, 0.027),
    ):
        yoke = model.part(f"{side_name}_yoke")
        yoke.visual(
            Cylinder(radius=0.0095, length=0.042),
            origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=dark_metal,
            name="fold_barrel",
        )
        yoke.visual(
            Box((0.014, 0.026, 0.026)),
            origin=Origin(xyz=(0.0, 0.0, -0.014)),
            material=dark_metal,
            name="fold_neck",
        )
        yoke.visual(
            Box((0.014, 0.128, 0.012)),
            origin=Origin(xyz=(0.0, 0.0, -0.032)),
            material=dark_metal,
            name="top_bridge",
        )
        yoke.visual(
            Box((0.010, 0.008, 0.106)),
            origin=Origin(xyz=(0.0, -0.058, -0.083)),
            material=dark_metal,
            name="front_arm",
        )
        yoke.visual(
            Cylinder(radius=0.012, length=0.014),
            origin=Origin(xyz=(0.0, -0.058, -YOKE_DROP), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=dark_metal,
            name="pivot_collar_front",
        )
        yoke.visual(
            Box((0.010, 0.008, 0.106)),
            origin=Origin(xyz=(0.0, 0.058, -0.083)),
            material=dark_metal,
            name="rear_arm",
        )
        yoke.visual(
            Cylinder(radius=0.012, length=0.014),
            origin=Origin(xyz=(0.0, 0.058, -YOKE_DROP), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=dark_metal,
            name="pivot_collar_rear",
        )

        model.articulation(
            f"{side_name}_fold",
            ArticulationType.REVOLUTE,
            parent=headband,
            child=yoke,
            origin=Origin(xyz=(sx, 0.0, FOLD_Z)),
            axis=fold_axis,
            motion_limits=MotionLimits(lower=0.0, upper=1.45, effort=4.0, velocity=2.0),
        )

        cup = model.part(f"{side_name}_cup")
        cup.visual(
            mesh_from_cadquery(
                _oval_disc(CUP_HALF_WIDTH, CUP_HALF_HEIGHT, CUP_THICKNESS),
                f"{side_name}_cup_shell",
                tolerance=0.0012,
            ),
            material=satin_black,
            name="oval_shell",
        )
        cup.visual(
            mesh_from_cadquery(
                _oval_ring(0.043, 0.059, 0.027, 0.041, 0.014),
                f"{side_name}_ear_cushion",
                tolerance=0.0012,
            ),
            origin=Origin(xyz=(cushion_x, 0.0, 0.0)),
            material=soft_pad,
            name="ear_cushion",
        )
        cup.visual(
            mesh_from_cadquery(
                _oval_disc(0.033, 0.047, 0.006),
                f"{side_name}_outer_plate",
                tolerance=0.0012,
            ),
            origin=Origin(xyz=(cover_x, 0.0, 0.0)),
            material=accent,
            name="outer_plate",
        )
        cup.visual(
            Cylinder(radius=0.0052, length=0.136),
            origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=dark_metal,
            name="pivot_pin",
        )
        cup.visual(
            Cylinder(radius=0.0026, length=0.004),
            origin=Origin(xyz=(cover_x + (0.002 if cover_x > 0 else -0.002), -0.020, -0.032), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=dark_metal,
            name="mic_port",
        )

        model.articulation(
            f"{side_name}_swivel",
            ArticulationType.REVOLUTE,
            parent=yoke,
            child=cup,
            origin=Origin(xyz=(0.0, 0.0, -YOKE_DROP)),
            axis=swivel_axis,
            motion_limits=MotionLimits(
                lower=0.0,
                upper=math.pi / 2.0,
                effort=2.0,
                velocity=2.5,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    headband = object_model.get_part("headband")

    left_yoke = object_model.get_part("left_yoke")
    right_yoke = object_model.get_part("right_yoke")
    left_cup = object_model.get_part("left_cup")
    right_cup = object_model.get_part("right_cup")

    left_fold = object_model.get_articulation("left_fold")
    right_fold = object_model.get_articulation("right_fold")
    left_swivel = object_model.get_articulation("left_swivel")
    right_swivel = object_model.get_articulation("right_swivel")

    # Captured hinge pins and cup trunnion pins are intentionally nested in
    # barrels/collars.  These scoped allowances are the mechanical retention,
    # not a broad cover for unrelated geometry.
    for side_name, yoke, cup in (
        ("left", left_yoke, left_cup),
        ("right", right_yoke, right_cup),
    ):
        ctx.allow_overlap(
            headband,
            yoke,
            elem_a=f"{side_name}_fold_pin",
            elem_b="fold_barrel",
            reason="The metal fold pin is intentionally captured through the yoke barrel.",
        )
        ctx.expect_within(
            headband,
            yoke,
            axes="xz",
            inner_elem=f"{side_name}_fold_pin",
            outer_elem="fold_barrel",
            margin=0.0,
            name=f"{side_name} fold pin centered in barrel",
        )
        ctx.expect_overlap(
            headband,
            yoke,
            axes="y",
            elem_a=f"{side_name}_fold_pin",
            elem_b="fold_barrel",
            min_overlap=0.036,
            name=f"{side_name} fold pin spans barrel",
        )

        for cue in ("front", "rear"):
            ctx.allow_overlap(
                cup,
                yoke,
                elem_a="pivot_pin",
                elem_b=f"pivot_collar_{cue}",
                reason="The cup pivot pin is intentionally clipped inside the yoke collar.",
            )
            ctx.expect_within(
                cup,
                yoke,
                axes="xz",
                inner_elem="pivot_pin",
                outer_elem=f"pivot_collar_{cue}",
                margin=0.0,
                name=f"{side_name} cup pin centered in {cue} collar",
            )
            ctx.expect_overlap(
                cup,
                yoke,
                axes="y",
                elem_a="pivot_pin",
                elem_b=f"pivot_collar_{cue}",
                min_overlap=0.010,
                name=f"{side_name} cup pin retained by {cue} collar",
            )
            ctx.allow_overlap(
                cup,
                yoke,
                elem_a="pivot_pin",
                elem_b=f"{cue}_arm",
                reason="The cup pivot pin passes through a simplified bored yoke arm.",
            )
            ctx.expect_overlap(
                cup,
                yoke,
                axes="xyz",
                elem_a="pivot_pin",
                elem_b=f"{cue}_arm",
                min_overlap=0.006,
                name=f"{side_name} cup pin passes through {cue} arm",
            )

    left_rest = ctx.part_world_position(left_cup)
    right_rest = ctx.part_world_position(right_cup)
    with ctx.pose({left_fold: 1.25, right_fold: 1.25}):
        left_folded = ctx.part_world_position(left_cup)
        right_folded = ctx.part_world_position(right_cup)
    ctx.check(
        "fold joints tuck cups inward",
        left_rest is not None
        and right_rest is not None
        and left_folded is not None
        and right_folded is not None
        and left_folded[0] > left_rest[0] + 0.055
        and right_folded[0] < right_rest[0] - 0.055,
        details=f"left rest/folded={left_rest}/{left_folded}, right rest/folded={right_rest}/{right_folded}",
    )

    left_rest_aabb = ctx.part_world_aabb(left_cup)
    with ctx.pose({left_swivel: math.pi / 2.0, right_swivel: math.pi / 2.0}):
        left_flat_aabb = ctx.part_world_aabb(left_cup)
        ctx.expect_overlap(
            left_cup,
            left_yoke,
            axes="y",
            elem_a="pivot_pin",
            elem_b="pivot_collar_front",
            min_overlap=0.010,
            name="left cup remains clipped while swiveled flat",
        )
        ctx.expect_overlap(
            right_cup,
            right_yoke,
            axes="y",
            elem_a="pivot_pin",
            elem_b="pivot_collar_rear",
            min_overlap=0.010,
            name="right cup remains clipped while swiveled flat",
        )
    ctx.check(
        "swivel makes cup lie flat",
        left_rest_aabb is not None
        and left_flat_aabb is not None
        and (left_rest_aabb[1][2] - left_rest_aabb[0][2]) > 0.11
        and (left_flat_aabb[1][2] - left_flat_aabb[0][2]) < 0.078,
        details=f"rest_aabb={left_rest_aabb}, flat_aabb={left_flat_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
