from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    BoxGeometry,
    Cylinder,
    CylinderGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    boolean_difference,
    mesh_from_geometry,
)

ASSETS = AssetContext.from_script(__file__)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, ASSETS.mesh_path(name))


def _ring_shell(
    *,
    outer_radius: float,
    inner_radius: float,
    height: float,
    z_center: float = 0.0,
    radial_segments: int = 64,
):
    outer = CylinderGeometry(radius=outer_radius, height=height, radial_segments=radial_segments)
    inner = CylinderGeometry(
        radius=inner_radius,
        height=height + 0.002,
        radial_segments=radial_segments,
    )
    return boolean_difference(outer, inner).translate(0.0, 0.0, z_center)


def _build_collar_core():
    collar = _ring_shell(
        outer_radius=0.0238,
        inner_radius=0.0189,
        height=0.028,
        z_center=0.014,
        radial_segments=72,
    )
    port_cut = BoxGeometry((0.018, 0.012, 0.010)).translate(-0.021, 0.0, 0.012)
    return boolean_difference(collar, port_cut)


def _add_knurl_rows(part, material, *, x_offset: float = 0.0) -> None:
    ridge_radius = 0.0230
    ridge_size = (0.0036, 0.0042, 0.007)
    row_centers = (0.009, 0.019)
    count = 20
    for row_index, z_center in enumerate(row_centers):
        angle_offset = (math.pi / count) if row_index else 0.0
        for index in range(count):
            angle = angle_offset + (2.0 * math.pi * index / count)
            part.visual(
                Box(ridge_size),
                origin=Origin(
                    xyz=(
                        x_offset + ridge_radius * math.cos(angle),
                        ridge_radius * math.sin(angle),
                        z_center,
                    ),
                    rpy=(0.0, 0.0, angle),
                ),
                material=material,
                name=f"knurl_{row_index}_{index:02d}",
            )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="mechanical_dropper_seatpost", assets=ASSETS)

    black_anodized = model.material("black_anodized", rgba=(0.09, 0.10, 0.11, 1.0))
    satin_black = model.material("satin_black", rgba=(0.15, 0.16, 0.18, 1.0))
    alloy = model.material("alloy", rgba=(0.72, 0.74, 0.77, 1.0))
    hardware = model.material("hardware", rgba=(0.26, 0.28, 0.31, 1.0))
    port_dark = model.material("port_dark", rgba=(0.04, 0.04, 0.05, 1.0))

    main_tube_mesh = _save_mesh(
        "dropper_main_tube.obj",
        _ring_shell(
            outer_radius=0.0158,
            inner_radius=0.0137,
            height=0.348,
            z_center=0.174,
            radial_segments=72,
        ),
    )
    upper_head_mesh = _save_mesh(
        "dropper_upper_head.obj",
        _ring_shell(
            outer_radius=0.0192,
            inner_radius=0.0142,
            height=0.072,
            z_center=0.384,
            radial_segments=72,
        ),
    )
    guide_bushing_mesh = _save_mesh(
        "dropper_guide_bushing.obj",
        _ring_shell(
            outer_radius=0.0142,
            inner_radius=0.0127,
            height=0.014,
            z_center=0.413,
            radial_segments=72,
        ),
    )
    seal_ring_mesh = _save_mesh(
        "dropper_seal_ring.obj",
        _ring_shell(
            outer_radius=0.0182,
            inner_radius=0.0129,
            height=0.010,
            z_center=0.425,
            radial_segments=72,
        ),
    )
    collar_core_mesh = _save_mesh("dropper_collar_core.obj", _build_collar_core())

    lower_body = model.part("lower_body")
    lower_body.visual(main_tube_mesh, material=black_anodized, name="main_tube")
    lower_body.visual(upper_head_mesh, material=satin_black, name="upper_head")
    lower_body.visual(guide_bushing_mesh, material=alloy, name="guide_bushing")
    lower_body.visual(seal_ring_mesh, material=hardware, name="seal_ring")
    lower_body.inertial = Inertial.from_geometry(
        Cylinder(radius=0.0195, length=0.430),
        mass=0.62,
        origin=Origin(xyz=(0.0, 0.0, 0.215)),
    )

    collar = model.part("collar")
    collar.visual(collar_core_mesh, origin=Origin(xyz=(0.019, 0.0, 0.0)), material=alloy, name="collar_ring")
    _add_knurl_rows(collar, alloy, x_offset=0.019)
    collar.inertial = Inertial.from_geometry(
        Cylinder(radius=0.024, length=0.028),
        mass=0.08,
        origin=Origin(xyz=(0.019, 0.0, 0.014)),
    )

    actuation_port = model.part("actuation_port")
    actuation_port.visual(
        Cylinder(radius=0.005, length=0.010),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=port_dark,
        name="port_sleeve",
    )
    actuation_port.visual(
        Box((0.005, 0.010, 0.008)),
        origin=Origin(xyz=(0.001, 0.0, 0.0)),
        material=hardware,
        name="port_stop",
    )
    actuation_port.inertial = Inertial.from_geometry(
        Box((0.014, 0.012, 0.012)),
        mass=0.015,
        origin=Origin(),
    )

    inner_tube = model.part("inner_tube")
    inner_tube.visual(
        Cylinder(radius=0.0127, length=0.320),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=black_anodized,
        name="stanchion",
    )
    inner_tube.visual(
        Cylinder(radius=0.0138, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.185)),
        material=hardware,
        name="top_cap",
    )
    inner_tube.inertial = Inertial.from_geometry(
        Cylinder(radius=0.0138, length=0.330),
        mass=0.34,
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
    )

    saddle_clamp = model.part("saddle_clamp")
    saddle_clamp.visual(
        Box((0.050, 0.018, 0.012)),
        origin=Origin(xyz=(-0.018, 0.0, 0.006)),
        material=satin_black,
        name="bridge_arm",
    )
    saddle_clamp.visual(
        Box((0.044, 0.024, 0.010)),
        origin=Origin(xyz=(-0.052, 0.0, 0.022)),
        material=satin_black,
        name="rear_cradle",
    )
    saddle_clamp.visual(
        Box((0.016, 0.018, 0.012)),
        origin=Origin(xyz=(-0.034, 0.0, 0.014)),
        material=satin_black,
        name="setback_yoke",
    )
    saddle_clamp.visual(
        Box((0.036, 0.018, 0.008)),
        origin=Origin(xyz=(-0.052, 0.0, 0.036)),
        material=alloy,
        name="upper_plate",
    )
    for x_pos, name_prefix in ((-0.064, "rear"), (-0.040, "front")):
        for side, y_pos in (("left", -0.012), ("right", 0.012)):
            saddle_clamp.visual(
                Cylinder(radius=0.003, length=0.020),
                origin=Origin(xyz=(x_pos, y_pos, 0.028)),
                material=hardware,
                name=f"{name_prefix}_bolt_{side}",
            )
    saddle_clamp.inertial = Inertial.from_geometry(
        Box((0.080, 0.032, 0.046)),
        mass=0.12,
        origin=Origin(xyz=(-0.040, 0.0, 0.023)),
    )

    model.articulation(
        "body_to_collar",
        ArticulationType.FIXED,
        parent=lower_body,
        child=collar,
        origin=Origin(xyz=(-0.019, 0.0, 0.392)),
    )
    model.articulation(
        "collar_to_port",
        ArticulationType.FIXED,
        parent=collar,
        child=actuation_port,
        origin=Origin(xyz=(-0.006, 0.0, 0.012)),
    )
    model.articulation(
        "stanchion_travel",
        ArticulationType.PRISMATIC,
        parent=lower_body,
        child=inner_tube,
        origin=Origin(xyz=(0.0, 0.0, 0.420)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=300.0,
            velocity=0.35,
            lower=0.0,
            upper=0.120,
        ),
    )
    model.articulation(
        "stanchion_to_clamp",
        ArticulationType.FIXED,
        parent=inner_tube,
        child=saddle_clamp,
        origin=Origin(xyz=(0.0, 0.0, 0.190)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    lower_body = object_model.get_part("lower_body")
    collar = object_model.get_part("collar")
    actuation_port = object_model.get_part("actuation_port")
    inner_tube = object_model.get_part("inner_tube")
    saddle_clamp = object_model.get_part("saddle_clamp")
    stanchion_travel = object_model.get_articulation("stanchion_travel")

    upper_head = lower_body.get_visual("upper_head")
    guide_bushing = lower_body.get_visual("guide_bushing")
    collar_ring = collar.get_visual("collar_ring")
    port_sleeve = actuation_port.get_visual("port_sleeve")
    stanchion = inner_tube.get_visual("stanchion")
    top_cap = inner_tube.get_visual("top_cap")
    bridge_arm = saddle_clamp.get_visual("bridge_arm")
    rear_cradle = saddle_clamp.get_visual("rear_cradle")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.warn_if_articulation_origin_near_geometry(tol=0.015)
    ctx.warn_if_part_geometry_disconnected()
    ctx.allow_overlap(
        inner_tube,
        lower_body,
        reason="stanchion runs through the lower body's guide bushing sleeve in the telescoping seatpost",
    )
    ctx.allow_overlap(
        actuation_port,
        collar,
        reason="recessed cable-stop sleeve nests inside the collar's lever actuation port",
    )
    ctx.check_articulation_overlaps(
        max_pose_samples=128,
        overlap_tol=0.001,
        overlap_volume_tol=0.0,
    )
    ctx.warn_if_overlaps(max_pose_samples=128, ignore_adjacent=True, ignore_fixed=True)

    ctx.expect_overlap(inner_tube, lower_body, axes="xy", min_overlap=0.020, elem_a=stanchion, elem_b=upper_head)
    ctx.expect_contact(inner_tube, lower_body, elem_a=stanchion, elem_b=guide_bushing)
    ctx.expect_origin_distance(inner_tube, lower_body, axes="xy", max_dist=0.001)
    ctx.expect_overlap(collar, lower_body, axes="xy", min_overlap=0.020, elem_a=collar_ring, elem_b=upper_head)
    ctx.expect_within(actuation_port, collar, axes="yz", inner_elem=port_sleeve, outer_elem=collar_ring)
    ctx.expect_gap(
        lower_body,
        actuation_port,
        axis="x",
        min_gap=0.0005,
        max_gap=0.0025,
        positive_elem=upper_head,
        negative_elem=port_sleeve,
    )
    ctx.expect_contact(saddle_clamp, inner_tube, elem_a=bridge_arm, elem_b=top_cap)
    ctx.expect_gap(
        inner_tube,
        saddle_clamp,
        axis="x",
        min_gap=0.012,
        positive_elem=stanchion,
        negative_elem=rear_cradle,
    )
    ctx.expect_gap(
        saddle_clamp,
        collar,
        axis="z",
        min_gap=0.18,
        max_gap=0.21,
        positive_elem=bridge_arm,
        negative_elem=collar_ring,
    )

    with ctx.pose({stanchion_travel: 0.120}):
        ctx.expect_overlap(
            inner_tube,
            lower_body,
            axes="xy",
            min_overlap=0.020,
            elem_a=stanchion,
            elem_b=upper_head,
        )
        ctx.expect_contact(inner_tube, lower_body, elem_a=stanchion, elem_b=guide_bushing)
        ctx.expect_origin_distance(inner_tube, lower_body, axes="xy", max_dist=0.001)
        ctx.expect_gap(
            saddle_clamp,
            collar,
            axis="z",
            min_gap=0.30,
            max_gap=0.33,
            positive_elem=bridge_arm,
            negative_elem=collar_ring,
        )
    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
