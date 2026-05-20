from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TireCarcass,
    TireGeometry,
    TireGroove,
    TireShoulder,
    TireSidewall,
    TireTread,
    WheelBore,
    WheelFace,
    WheelGeometry,
    WheelHub,
    WheelRim,
    WheelSpokes,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _superellipse_loop(width: float, depth: float, z: float, segments: int = 64, exponent: float = 4.0):
    """Rounded-rectangle/superellipse loop in the local XY plane."""
    pts = []
    for i in range(segments):
        t = 2.0 * math.pi * i / segments
        c = math.cos(t)
        s = math.sin(t)
        x = math.copysign(abs(c) ** (2.0 / exponent), c) * width * 0.5
        y = math.copysign(abs(s) ** (2.0 / exponent), s) * depth * 0.5
        pts.append((x, y, z))
    return pts


def _add_loop(geom: MeshGeometry, pts):
    return [geom.add_vertex(x, y, z) for x, y, z in pts]


def _quad(geom: MeshGeometry, a: int, b: int, c: int, d: int) -> None:
    geom.add_face(a, b, c)
    geom.add_face(a, c, d)


def _cap_loop(geom: MeshGeometry, loop, center_xyz, reverse: bool = False) -> None:
    center = geom.add_vertex(*center_xyz)
    n = len(loop)
    for i in range(n):
        a = loop[i]
        b = loop[(i + 1) % n]
        if reverse:
            geom.add_face(center, b, a)
        else:
            geom.add_face(center, a, b)


def _hollow_bin_body_mesh() -> MeshGeometry:
    """Open-topped, thick-walled, tapered molded wheelie-bin body."""
    geom = MeshGeometry()
    wall = 0.030
    # z, outside width, outside depth.  The wider top and rounded corners are
    # characteristic of blow/injection molded municipal bins.
    sections = [
        (0.070, 0.410, 0.400),
        (0.170, 0.450, 0.485),
        (0.540, 0.540, 0.620),
        (0.915, 0.610, 0.715),
    ]
    outer = [_add_loop(geom, _superellipse_loop(w, d, z)) for z, w, d in sections]

    # Interior starts above a structural floor and stays visibly open under the
    # lid.  A slightly lower exponent makes the inner corners look radiused.
    inner_sections = [
        (0.130, 0.350, 0.340),
        (0.240, 0.405, 0.440),
        (0.560, 0.485, 0.560),
        (0.915, 0.610 - 2 * wall, 0.715 - 2 * wall),
    ]
    inner = [_add_loop(geom, _superellipse_loop(w, d, z, exponent=3.3)) for z, w, d in inner_sections]

    n = len(outer[0])
    # Outer and inner tapered walls.
    for loops in (outer,):
        for lower, upper in zip(loops, loops[1:]):
            for j in range(n):
                _quad(geom, lower[j], lower[(j + 1) % n], upper[(j + 1) % n], upper[j])
    for lower, upper in zip(inner, inner[1:]):
        for j in range(n):
            # Reverse winding so normals face the cavity.
            _quad(geom, lower[j], upper[j], upper[(j + 1) % n], lower[(j + 1) % n])

    # Thick top rim: ring between outside and inside, leaving an actual opening.
    outer_top = outer[-1]
    inner_top = inner[-1]
    for j in range(n):
        _quad(geom, inner_top[j], inner_top[(j + 1) % n], outer_top[(j + 1) % n], outer_top[j])

    # Underside and internal floor make the body read as a durable hollow tub.
    _cap_loop(geom, outer[0], (0.0, 0.0, sections[0][0]), reverse=True)
    _cap_loop(geom, inner[0], (0.0, 0.0, inner_sections[0][0]), reverse=False)
    return geom


def _rounded_extrusion(width: float, depth: float, radius: float, height: float, *, z0: float) -> MeshGeometry:
    geom = ExtrudeGeometry.from_z0(
        rounded_rect_profile(width, depth, radius, corner_segments=10),
        height,
        cap=True,
        closed=True,
    )
    geom.translate(0.0, 0.0, z0)
    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="realistic_wheelie_bin")

    green = model.material("molded_dark_green", rgba=(0.02, 0.24, 0.12, 1.0))
    green_highlight = model.material("slightly_worn_green", rgba=(0.035, 0.32, 0.16, 1.0))
    black = model.material("black_rubber", rgba=(0.005, 0.005, 0.004, 1.0))
    charcoal = model.material("charcoal_plastic", rgba=(0.05, 0.055, 0.055, 1.0))
    axle_grey = model.material("galvanized_axle_grey", rgba=(0.46, 0.48, 0.46, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_geometry(_hollow_bin_body_mesh(), "body_shell"),
        material=green,
        name="body_shell",
    )

    # Raised molded front panel, vertical stiffening ribs, lower foot rail, and
    # rear handle brackets are all slightly embedded into the shell as one molded
    # plastic body rather than floating decorations.
    body.visual(
        Box((0.385, 0.050, 0.410)),
        origin=Origin(xyz=(0.0, -0.312, 0.545)),
        material=green_highlight,
        name="front_label_panel",
    )
    for x in (-0.245, 0.245):
        body.visual(
            Box((0.045, 0.055, 0.560)),
            origin=Origin(xyz=(x, -0.313, 0.505)),
            material=green_highlight,
            name=f"front_stiffener_{'neg' if x < 0 else 'pos'}",
        )
    for x in (-0.302, 0.302):
        body.visual(
            Box((0.030, 0.255, 0.520)),
            origin=Origin(xyz=(x, 0.010, 0.520)),
            material=green_highlight,
            name=f"side_molded_rib_{'neg' if x < 0 else 'pos'}",
        )

    body.visual(
        Cylinder(radius=0.018, length=0.650),
        origin=Origin(xyz=(0.0, 0.318, 0.125), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=axle_grey,
        name="axle",
    )
    for x in (-0.245, 0.245):
        body.visual(
            Box((0.092, 0.145, 0.125)),
            origin=Origin(xyz=(x, 0.255, 0.145)),
            material=green,
            name=f"axle_bracket_{'neg' if x < 0 else 'pos'}",
        )
    body.visual(
        Box((0.470, 0.120, 0.060)),
        origin=Origin(xyz=(0.0, 0.340, 0.078)),
        material=charcoal,
        name="tilt_foot_rail",
    )

    body.visual(
        Cylinder(radius=0.025, length=0.565),
        origin=Origin(xyz=(0.0, 0.410, 0.875), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=green_highlight,
        name="rear_handle_bar",
    )
    for x in (-0.235, 0.235):
        body.visual(
            Box((0.070, 0.090, 0.260)),
            origin=Origin(xyz=(x, 0.375, 0.790)),
            material=green,
            name=f"handle_support_{'neg' if x < 0 else 'pos'}",
        )

    for x in (-0.215, 0.215):
        body.visual(
            Box((0.145, 0.040, 0.070)),
            origin=Origin(xyz=(x, 0.370, 0.920)),
            material=green,
            name=f"hinge_riser_{'neg' if x < 0 else 'pos'}",
        )
        body.visual(
            Cylinder(radius=0.026, length=0.108),
            origin=Origin(xyz=(x, 0.376, 0.945), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=charcoal,
            name=f"body_hinge_barrel_{'neg' if x < 0 else 'pos'}",
        )

    lid = model.part("lid")
    lid_panel = _rounded_extrusion(0.665, 0.690, 0.055, 0.036, z0=0.006)
    lid_panel.translate(0.0, -0.375, 0.0)
    lid.visual(
        mesh_from_geometry(lid_panel, "lid_panel"),
        material=green,
        name="lid_panel",
    )
    raised = _rounded_extrusion(0.500, 0.445, 0.050, 0.015, z0=0.038)
    raised.translate(0.0, -0.355, 0.0)
    lid.visual(
        mesh_from_geometry(raised, "raised_lid_panel"),
        material=green_highlight,
        name="raised_lid_panel",
    )
    lid.visual(
        Box((0.660, 0.070, 0.086)),
        origin=Origin(xyz=(0.0, -0.735, 0.010)),
        material=green,
        name="front_lip",
    )
    for x in (-0.352, 0.352):
        lid.visual(
            Box((0.052, 0.675, 0.065)),
            origin=Origin(xyz=(x, -0.385, 0.008)),
            material=green,
            name=f"side_lip_{'neg' if x < 0 else 'pos'}",
        )
    # Lid knuckles occupy the spaces between the fixed body knuckles.
    for x, length, suffix in ((0.0, 0.220, "center"), (-0.330, 0.078, "neg"), (0.330, 0.078, "pos")):
        lid.visual(
            Cylinder(radius=0.024, length=length),
            origin=Origin(xyz=(x, 0.000, 0.000), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=charcoal,
            name=f"lid_hinge_barrel_{suffix}",
        )
    for x, width, suffix in ((0.0, 0.245, "center"), (-0.330, 0.090, "neg"), (0.330, 0.090, "pos")):
        lid.visual(
            Box((width, 0.036, 0.030)),
            origin=Origin(xyz=(x, -0.026, 0.020)),
            material=green,
            name=f"rear_lid_tab_{suffix}",
        )

    wheel_mesh = mesh_from_geometry(
        WheelGeometry(
            0.094,
            0.052,
            rim=WheelRim(inner_radius=0.058, flange_height=0.007, flange_thickness=0.004),
            hub=WheelHub(radius=0.032, width=0.044, cap_style="domed"),
            face=WheelFace(dish_depth=0.006, front_inset=0.002, rear_inset=0.002),
            spokes=WheelSpokes(style="straight", count=6, thickness=0.006, window_radius=0.010),
            bore=WheelBore(style="round", diameter=0.018),
        ),
        "wheel_rim",
    )
    tire_mesh = mesh_from_geometry(
        TireGeometry(
            0.125,
            0.070,
            inner_radius=0.094,
            carcass=TireCarcass(belt_width_ratio=0.72, sidewall_bulge=0.030),
            tread=TireTread(style="block", depth=0.006, count=22, land_ratio=0.56),
            grooves=(TireGroove(center_offset=0.0, width=0.007, depth=0.003),),
            sidewall=TireSidewall(style="square", bulge=0.018),
            shoulder=TireShoulder(width=0.008, radius=0.004),
        ),
        "wheel_tire",
    )
    wheel_centers = [(-0.337, 0.318, 0.125), (0.337, 0.318, 0.125)]
    for idx, xyz in enumerate(wheel_centers):
        wheel = model.part(f"wheel_{idx}")
        wheel.visual(tire_mesh, material=black, name="tire")
        wheel.visual(wheel_mesh, material=charcoal, name="rim")
        model.articulation(
            f"body_to_wheel_{idx}",
            ArticulationType.CONTINUOUS,
            parent=body,
            child=wheel,
            origin=Origin(xyz=xyz),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=12.0, velocity=18.0),
        )

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, 0.376, 0.945)),
        # The lid panel extends forward along local -Y from the hinge, so -X
        # makes positive q lift the free/front edge upward and back.
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=3.0, lower=0.0, upper=1.85),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    lid_hinge = object_model.get_articulation("body_to_lid")

    with ctx.pose({lid_hinge: 0.0}):
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            min_gap=0.010,
            max_gap=0.055,
            positive_elem="lid_panel",
            negative_elem="body_shell",
            name="closed lid sits just above the molded rim",
        )
        ctx.expect_overlap(
            lid,
            body,
            axes="xy",
            min_overlap=0.42,
            elem_a="lid_panel",
            elem_b="body_shell",
            name="closed lid covers the bin opening",
        )

    closed_front = ctx.part_element_world_aabb(lid, elem="front_lip")
    with ctx.pose({lid_hinge: 1.45}):
        open_front = ctx.part_element_world_aabb(lid, elem="front_lip")
    ctx.check(
        "lid hinge lifts the front edge",
        closed_front is not None
        and open_front is not None
        and open_front[0][2] > closed_front[1][2] + 0.25,
        details=f"closed_front={closed_front}, open_front={open_front}",
    )

    for idx in (0, 1):
        wheel = object_model.get_part(f"wheel_{idx}")
        wheel_joint = object_model.get_articulation(f"body_to_wheel_{idx}")
        ctx.allow_overlap(
            body,
            wheel,
            elem_a="axle",
            elem_b="rim",
            reason="The metal axle is intentionally captured through the molded wheel hub bore.",
        )
        ctx.expect_overlap(
            body,
            wheel,
            axes="x",
            min_overlap=0.006,
            elem_a="axle",
            elem_b="rim",
            name=f"wheel_{idx} hub remains captured on the axle",
        )
        ctx.expect_origin_gap(
            wheel,
            body,
            axis="z",
            min_gap=0.110,
            max_gap=0.140,
            name=f"wheel_{idx} axle is at wheel center height",
        )
        at_rest = ctx.part_world_aabb(wheel)
        with ctx.pose({wheel_joint: math.pi / 2.0}):
            spun = ctx.part_world_aabb(wheel)
        ctx.check(
            f"wheel_{idx} spins about the rear axle",
            at_rest is not None
            and spun is not None
            and abs((at_rest[0][2] + at_rest[1][2]) - (spun[0][2] + spun[1][2])) < 0.010,
            details=f"rest={at_rest}, spun={spun}",
        )

    return ctx.report()


object_model = build_object_model()
