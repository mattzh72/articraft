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
    Cylinder,
    MeshGeometry,
    MotionLimits,
    Origin,
    TorusGeometry,
    TestContext,
    TestReport,
    mesh_from_geometry,
    repair_loft,
    section_loft,
    superellipse_profile,
)

ASSETS = AssetContext.from_script(__file__)
REST_IPD_YAW = math.radians(14.0)


def _save_mesh(name: str, geometry: MeshGeometry):
    return mesh_from_geometry(geometry, ASSETS.mesh_path(name))


def _merge_geometries(geometries: list[MeshGeometry]) -> MeshGeometry:
    merged = MeshGeometry()
    for geometry in geometries:
        merged.merge(geometry)
    return merged


def _yz_section(
    x_pos: float,
    y_center: float,
    z_center: float,
    width: float,
    height: float,
    *,
    exponent: float,
    segments: int = 44,
) -> list[tuple[float, float, float]]:
    profile = superellipse_profile(width, height, exponent=exponent, segments=segments)
    return [(x_pos, y_center + y, z_center + z) for y, z in profile]


def _half_shell_geometry(side_sign: float) -> MeshGeometry:
    sections = [
        _yz_section(-0.044, side_sign * 0.054, 0.045, 0.034, 0.048, exponent=2.3),
        _yz_section(-0.012, side_sign * 0.052, 0.046, 0.044, 0.062, exponent=2.6),
        _yz_section(0.018, side_sign * 0.058, 0.046, 0.058, 0.080, exponent=3.0),
        _yz_section(0.048, side_sign * 0.066, 0.042, 0.062, 0.074, exponent=2.8),
        _yz_section(0.074, side_sign * 0.070, 0.040, 0.050, 0.060, exponent=2.3),
    ]
    return repair_loft(section_loft(sections))


def _knurl_band_geometry(
    radius: float,
    length: float,
    tooth_depth: float,
    tooth_count: int,
) -> MeshGeometry:
    base = MeshGeometry()
    half = length * 0.5
    for index in range(tooth_count):
        angle = math.tau * index / tooth_count
        next_angle = math.tau * (index + 1) / tooth_count
        r0 = radius
        r1 = radius + tooth_depth
        if index % 2:
            r0, r1 = r1, r0
        y0 = r0 * math.cos(angle)
        z0 = r0 * math.sin(angle)
        y1 = r1 * math.cos(next_angle)
        z1 = r1 * math.sin(next_angle)
        y2 = r1 * math.cos(angle)
        z2 = r1 * math.sin(angle)
        y3 = r0 * math.cos(next_angle)
        z3 = r0 * math.sin(next_angle)
        a = base.add_vertex(-half, y0, z0)
        b = base.add_vertex(-half, y1, z1)
        c = base.add_vertex(half, y1, z1)
        d = base.add_vertex(half, y0, z0)
        e = base.add_vertex(-half, y2, z2)
        f = base.add_vertex(half, y2, z2)
        g = base.add_vertex(-half, y3, z3)
        h = base.add_vertex(half, y3, z3)
        base.add_face(a, b, c)
        base.add_face(a, c, d)
        base.add_face(a, e, f)
        base.add_face(a, f, d)
        base.add_face(b, g, h)
        base.add_face(b, h, c)
        base.add_face(e, g, b)
        base.add_face(e, b, a)
        base.add_face(d, c, h)
        base.add_face(d, h, f)
    return base


def _focus_knob_geometry() -> MeshGeometry:
    knob = _knurl_band_geometry(radius=0.016, length=0.026, tooth_depth=0.0018, tooth_count=28)
    hub = _knurl_band_geometry(radius=0.011, length=0.034, tooth_depth=0.0009, tooth_count=18)
    return _merge_geometries([knob, hub])


def _diopter_ring_geometry() -> MeshGeometry:
    ring = TorusGeometry(radius=0.0290, tube=0.0010, radial_segments=18, tubular_segments=42)
    ring.rotate_y(math.pi / 2.0)
    return ring


def _add_half_body_visuals(part, *, side_sign: float, shell_mesh_name: str, armor, metal, glass) -> None:
    part.visual(
        Box((0.018, 0.012, 0.018)),
        origin=Origin(xyz=(0.000, side_sign * 0.012, 0.024)),
        material=metal,
        name="hinge_lug",
    )
    part.visual(
        Box((0.040, 0.016, 0.020)),
        origin=Origin(xyz=(0.004, side_sign * 0.0245, 0.024)),
        material=armor,
        name="lower_bridge_arm",
    )
    part.visual(
        Box((0.028, 0.014, 0.016)),
        origin=Origin(xyz=(-0.016, side_sign * 0.040, 0.042)),
        material=armor,
        name="upper_bridge_arm",
    )
    part.visual(
        _save_mesh(shell_mesh_name, _half_shell_geometry(side_sign)),
        material=armor,
        name="armor_shell",
    )
    part.visual(
        Box((0.038, 0.020, 0.020)),
        origin=Origin(xyz=(0.044, side_sign * 0.068, 0.026)),
        material=armor,
        name="objective_saddle",
    )
    part.visual(
        Box((0.018, 0.018, 0.016)),
        origin=Origin(xyz=(-0.028, side_sign * 0.060, 0.050)),
        material=armor,
        name="ocular_saddle",
    )
    part.visual(
        Cylinder(radius=0.034, length=0.058),
        origin=Origin(
            xyz=(0.092, side_sign * 0.072, 0.030),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=armor,
        name="objective_armor",
    )
    part.visual(
        Cylinder(radius=0.038, length=0.008),
        origin=Origin(
            xyz=(0.125, side_sign * 0.072, 0.030),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=metal,
        name="objective_lip",
    )
    part.visual(
        Cylinder(radius=0.025, length=0.003),
        origin=Origin(
            xyz=(0.130, side_sign * 0.072, 0.030),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=glass,
        name="objective_lens",
    )
    part.visual(
        Cylinder(radius=0.020, length=0.022),
        origin=Origin(
            xyz=(-0.031, side_sign * 0.072, 0.048),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=armor,
        name="ocular_neck",
    )
    part.visual(
        Cylinder(radius=0.025, length=0.036),
        origin=Origin(
            xyz=(-0.060, side_sign * 0.072, 0.048),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=armor,
        name="ocular_housing",
    )
    part.visual(
        Cylinder(radius=0.029, length=0.020),
        origin=Origin(
            xyz=(-0.087, side_sign * 0.072, 0.048),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=armor,
        name="eyecup",
    )
    part.visual(
        Cylinder(radius=0.017, length=0.004),
        origin=Origin(
            xyz=(-0.099, side_sign * 0.072, 0.048),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=glass,
        name="ocular_lens",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="marine_7x50_porro_binocular", assets=ASSETS)

    armor = model.material("rubber_armor", rgba=(0.12, 0.13, 0.14, 1.0))
    armor_dark = model.material("rubber_trim", rgba=(0.08, 0.09, 0.10, 1.0))
    metal = model.material("dark_metal", rgba=(0.30, 0.32, 0.35, 1.0))
    glass = model.material("coated_glass", rgba=(0.45, 0.58, 0.68, 0.42))

    bridge = model.part("bridge")
    bridge.visual(
        Cylinder(radius=0.0055, length=0.042),
        origin=Origin(xyz=(0.0, 0.0, 0.024)),
        material=metal,
        name="hinge_spindle",
    )
    bridge.visual(
        Box((0.020, 0.006, 0.018)),
        origin=Origin(xyz=(-0.004, 0.0, 0.024)),
        material=armor_dark,
        name="hinge_block",
    )
    bridge.visual(
        Box((0.016, 0.004, 0.016)),
        origin=Origin(xyz=(-0.018, 0.0, 0.030)),
        material=armor_dark,
        name="focus_bridge",
    )
    bridge.visual(
        Box((0.016, 0.008, 0.0244)),
        origin=Origin(xyz=(-0.032, 0.0, 0.034)),
        material=armor_dark,
        name="focus_tower",
    )

    left_body = model.part("left_body")
    _add_half_body_visuals(
        left_body,
        side_sign=1.0,
        shell_mesh_name="left_body_shell.obj",
        armor=armor,
        metal=metal,
        glass=glass,
    )

    right_body = model.part("right_body")
    _add_half_body_visuals(
        right_body,
        side_sign=-1.0,
        shell_mesh_name="right_body_shell.obj",
        armor=armor,
        metal=metal,
        glass=glass,
    )
    right_body.visual(
        Cylinder(radius=0.029, length=0.004),
        origin=Origin(
            xyz=(-0.079, -0.072, 0.048),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=metal,
        name="diopter_stop",
    )

    focus_knob = model.part("focus_knob")
    focus_knob.visual(
        Cylinder(radius=0.016, length=0.022),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=armor_dark,
        name="focus_wheel",
    )
    focus_knob.visual(
        Cylinder(radius=0.010, length=0.028),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=metal,
        name="focus_hub",
    )

    diopter_ring = model.part("diopter_ring")
    diopter_ring.visual(
        _save_mesh("diopter_ring.obj", _diopter_ring_geometry()),
        material=armor_dark,
        name="diopter_band",
    )

    model.articulation(
        "bridge_to_left_body",
        ArticulationType.REVOLUTE,
        parent=bridge,
        child=left_body,
        origin=Origin(rpy=(0.0, 0.0, REST_IPD_YAW)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=6.0, velocity=1.2, lower=-0.10, upper=0.10),
    )
    model.articulation(
        "bridge_to_right_body",
        ArticulationType.REVOLUTE,
        parent=bridge,
        child=right_body,
        origin=Origin(rpy=(0.0, 0.0, -REST_IPD_YAW)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=6.0, velocity=1.2, lower=-0.10, upper=0.10),
    )
    model.articulation(
        "bridge_to_focus_knob",
        ArticulationType.REVOLUTE,
        parent=bridge,
        child=focus_knob,
        origin=Origin(xyz=(-0.032, 0.0, 0.0622)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.6, velocity=6.0, lower=-1.5, upper=1.5),
    )
    model.articulation(
        "right_body_to_diopter_ring",
        ArticulationType.REVOLUTE,
        parent=right_body,
        child=diopter_ring,
        origin=Origin(xyz=(-0.076, -0.072, 0.048)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.2, velocity=2.0, lower=-0.4, upper=0.4),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root, seed=0)
    bridge = object_model.get_part("bridge")
    left_body = object_model.get_part("left_body")
    right_body = object_model.get_part("right_body")
    focus_knob = object_model.get_part("focus_knob")
    diopter_ring = object_model.get_part("diopter_ring")

    left_hinge = object_model.get_articulation("bridge_to_left_body")
    right_hinge = object_model.get_articulation("bridge_to_right_body")
    focus_joint = object_model.get_articulation("bridge_to_focus_knob")
    diopter_joint = object_model.get_articulation("right_body_to_diopter_ring")

    hinge_spindle = bridge.get_visual("hinge_spindle")
    focus_tower = bridge.get_visual("focus_tower")
    left_collar = left_body.get_visual("hinge_lug")
    right_collar = right_body.get_visual("hinge_lug")
    left_shell = left_body.get_visual("armor_shell")
    right_shell = right_body.get_visual("armor_shell")
    left_objective = left_body.get_visual("objective_armor")
    right_objective = right_body.get_visual("objective_armor")
    left_eyecup = left_body.get_visual("eyecup")
    right_eyecup = right_body.get_visual("eyecup")
    right_ocular = right_body.get_visual("ocular_housing")
    focus_wheel = focus_knob.get_visual("focus_wheel")
    focus_hub = focus_knob.get_visual("focus_hub")
    diopter_band = diopter_ring.get_visual("diopter_band")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_isolated_parts(contact_tol=0.005)
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose(overlap_tol=0.0005)
    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=48,
        overlap_tol=0.0005,
        ignore_adjacent=False,
        ignore_fixed=True,
    )

    ctx.expect_overlap(
        left_body,
        bridge,
        axes="xz",
        min_overlap=0.008,
        elem_a=left_collar,
        elem_b=hinge_spindle,
        name="left hinge lug is seated on the spindle",
    )
    ctx.expect_overlap(
        right_body,
        bridge,
        axes="xz",
        min_overlap=0.008,
        elem_a=right_collar,
        elem_b=hinge_spindle,
        name="right hinge lug is seated on the spindle",
    )
    ctx.expect_gap(
        left_body,
        right_body,
        axis="y",
        min_gap=0.055,
        positive_elem=left_objective,
        negative_elem=right_objective,
        name="objective barrels remain clearly separated",
    )
    ctx.expect_gap(
        left_body,
        right_body,
        axis="y",
        min_gap=0.020,
        positive_elem=left_eyecup,
        negative_elem=right_eyecup,
        name="eyepiece spacing stays binocular-like",
    )
    ctx.expect_gap(
        left_body,
        right_body,
        axis="y",
        max_gap=0.055,
        max_penetration=0.0,
        positive_elem=left_eyecup,
        negative_elem=right_eyecup,
        name="porro eyepieces stay closer together than the objectives",
    )
    ctx.expect_contact(
        focus_knob,
        bridge,
        contact_tol=0.0001,
        elem_a=focus_wheel,
        elem_b=focus_tower,
        name="focus knob rests on the bridge tower",
    )
    ctx.expect_gap(
        focus_knob,
        bridge,
        axis="z",
        min_gap=-1e-5,
        max_gap=0.0006,
        positive_elem=focus_wheel,
        negative_elem=focus_tower,
        name="focus knob rides on the tower pedestal",
    )
    ctx.expect_overlap(
        focus_knob,
        bridge,
        axes="xy",
        min_overlap=0.008,
        elem_a=focus_hub,
        elem_b=focus_tower,
        name="focus knob stays centered on the bridge tower",
    )
    ctx.expect_overlap(
        diopter_ring,
        right_body,
        axes="yz",
        min_overlap=0.020,
        elem_a=diopter_band,
        elem_b=right_ocular,
        name="diopter ring aligns with the right eyepiece",
    )
    ctx.expect_contact(
        diopter_ring,
        right_body,
        contact_tol=0.0001,
        elem_a=diopter_band,
        elem_b=right_eyecup,
        name="diopter ring seats on the right eyecup",
    )
    ctx.expect_gap(
        left_body,
        focus_knob,
        axis="y",
        min_gap=0.006,
        positive_elem=left_shell,
        negative_elem=focus_wheel,
        name="focus knob clears the left barrel shoulder",
    )
    ctx.expect_gap(
        focus_knob,
        right_body,
        axis="y",
        min_gap=0.006,
        positive_elem=focus_wheel,
        negative_elem=right_shell,
        name="focus knob clears the right barrel shoulder",
    )

    with ctx.pose({left_hinge: 0.08, right_hinge: -0.08}):
        ctx.expect_gap(
            left_body,
            right_body,
            axis="y",
            min_gap=0.070,
            positive_elem=left_objective,
            negative_elem=right_objective,
            name="opened hinge widens the objective spacing",
        )

    with ctx.pose({left_hinge: -0.08, right_hinge: 0.08}):
        ctx.expect_gap(
            left_body,
            right_body,
            axis="y",
            min_gap=0.010,
            positive_elem=left_eyecup,
            negative_elem=right_eyecup,
            name="closed hinge still keeps the eyecups distinct",
        )

    with ctx.pose({focus_joint: 1.0, diopter_joint: 0.2}):
        ctx.expect_contact(
            focus_knob,
            bridge,
            contact_tol=0.0001,
            elem_a=focus_wheel,
            elem_b=focus_tower,
            name="focus wheel stays supported while turned",
        )
        ctx.expect_overlap(
            diopter_ring,
            right_body,
            axes="yz",
            min_overlap=0.020,
            elem_a=diopter_band,
            elem_b=right_ocular,
            name="diopter ring remains concentric while adjusted",
        )

    for articulation in (left_hinge, right_hinge, focus_joint, diopter_joint):
        limits = articulation.motion_limits
        if limits is None or limits.lower is None or limits.upper is None:
            continue
        with ctx.pose({articulation: limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(
                overlap_tol=0.0005,
                name=f"{articulation.name}_lower_no_overlap",
            )
            ctx.fail_if_isolated_parts(
                contact_tol=0.005,
                name=f"{articulation.name}_lower_no_floating",
            )
        with ctx.pose({articulation: limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(
                overlap_tol=0.0005,
                name=f"{articulation.name}_upper_no_overlap",
            )
            ctx.fail_if_isolated_parts(
                contact_tol=0.005,
                name=f"{articulation.name}_upper_no_floating",
            )
    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
