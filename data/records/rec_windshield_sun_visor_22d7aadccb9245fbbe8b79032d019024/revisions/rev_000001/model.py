from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _visor_panel_mesh() -> MeshGeometry:
    """A broad, slightly crowned and drooped exterior sun-visor shell."""
    geom = MeshGeometry()
    x0 = 0.055
    length = 0.745
    root_width = 1.12
    front_width = 1.28
    thickness = 0.024
    x_segments = 12
    y_fracs = (-1.0, -0.55, 0.0, 0.55, 1.0)

    top: list[list[int]] = []
    bottom: list[list[int]] = []
    for i in range(x_segments + 1):
        t = i / x_segments
        x = x0 + length * t
        width = root_width + (front_width - root_width) * t
        top_row = []
        bottom_row = []
        for yf in y_fracs:
            y = yf * width * 0.5
            crown = 0.007 * (1.0 - yf * yf)
            z_top = -0.033 - 0.078 * t + crown
            top_row.append(geom.add_vertex(x, y, z_top))
            bottom_row.append(geom.add_vertex(x, y, z_top - thickness))
        top.append(top_row)
        bottom.append(bottom_row)

    rows = len(y_fracs)
    for i in range(x_segments):
        for j in range(rows - 1):
            geom.add_face(top[i][j], top[i + 1][j], top[i + 1][j + 1])
            geom.add_face(top[i][j], top[i + 1][j + 1], top[i][j + 1])
            geom.add_face(bottom[i][j + 1], bottom[i + 1][j + 1], bottom[i + 1][j])
            geom.add_face(bottom[i][j + 1], bottom[i + 1][j], bottom[i][j])

    for i in range(x_segments):
        # Side walls at the rolled-gutter edges.
        geom.add_face(top[i][0], bottom[i][0], bottom[i + 1][0])
        geom.add_face(top[i][0], bottom[i + 1][0], top[i + 1][0])
        geom.add_face(top[i + 1][-1], bottom[i + 1][-1], bottom[i][-1])
        geom.add_face(top[i + 1][-1], bottom[i][-1], top[i][-1])

    for j in range(rows - 1):
        # Rear and front closed edges.
        geom.add_face(top[0][j + 1], bottom[0][j + 1], bottom[0][j])
        geom.add_face(top[0][j + 1], bottom[0][j], top[0][j])
        geom.add_face(top[-1][j], bottom[-1][j], bottom[-1][j + 1])
        geom.add_face(top[-1][j], bottom[-1][j + 1], top[-1][j + 1])

    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="weatherproof_windshield_sun_visor")

    powder_black = model.material("powder_black", rgba=(0.02, 0.025, 0.025, 1.0))
    dark_gasket = model.material("dark_epdm_gasket", rgba=(0.005, 0.006, 0.006, 1.0))
    smoked_poly = model.material("smoked_polycarbonate", rgba=(0.12, 0.15, 0.17, 0.78))
    stainless = model.material("brushed_stainless", rgba=(0.72, 0.72, 0.68, 1.0))
    sealant = model.material("gray_sealed_edges", rgba=(0.08, 0.085, 0.08, 1.0))

    roof = model.part("roof_header")
    roof.visual(
        Box((0.18, 1.42, 0.012)),
        origin=Origin(xyz=(-0.125, 0.0, -0.071)),
        material=dark_gasket,
        name="roof_gasket",
    )
    roof.visual(
        Box((0.18, 1.36, 0.035)),
        origin=Origin(xyz=(-0.125, 0.0, -0.0475)),
        material=powder_black,
        name="roof_plate",
    )
    roof.visual(
        Box((0.030, 1.40, 0.018)),
        origin=Origin(xyz=(-0.030, 0.0, -0.081)),
        material=dark_gasket,
        name="front_drip_overhang",
    )
    roof.visual(
        Box((0.130, 1.32, 0.028)),
        origin=Origin(xyz=(-0.035, 0.0, 0.049)),
        material=powder_black,
        name="hinge_rain_cowl",
    )
    roof.visual(
        Box((0.026, 1.32, 0.086)),
        origin=Origin(xyz=(-0.088, 0.0, 0.003)),
        material=powder_black,
        name="cowl_back_wall",
    )

    for idx, y in enumerate((-0.54, 0.0, 0.54)):
        length = 0.20 if idx != 1 else 0.28
        roof.visual(
            Cylinder(radius=0.033, length=length),
            origin=Origin(xyz=(0.0, y, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=powder_black,
            name=f"primary_knuckle_{idx}",
        )
        roof.visual(
            Box((0.044, length, 0.060)),
            origin=Origin(xyz=(-0.055, y, -0.030)),
            material=powder_black,
            name=f"hinge_saddle_{idx}",
        )

    screw_id = 0
    for x in (-0.170, -0.080):
        for y in (-0.55, -0.18, 0.18, 0.55):
            roof.visual(
                Cylinder(radius=0.014, length=0.006),
                origin=Origin(xyz=(x, y, -0.0275), rpy=(0.0, 0.0, 0.0)),
                material=stainless,
                name=f"roof_screw_{screw_id}",
            )
            screw_id += 1

    arm = model.part("hinge_arm")
    arm.visual(
        Cylinder(radius=0.014, length=1.34),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=stainless,
        name="primary_pin",
    )
    for y in (-0.31, 0.31):
        arm.visual(
            Box((0.055, 0.080, 0.070)),
            origin=Origin(xyz=(0.018, y, -0.025)),
            material=powder_black,
            name=f"primary_lug_{'neg' if y < 0 else 'pos'}",
        )
        arm.visual(
            Box((0.455, 0.060, 0.038)),
            origin=Origin(xyz=(0.245, y, -0.058)),
            material=powder_black,
            name=f"roof_arm_{'neg' if y < 0 else 'pos'}",
        )
        arm.visual(
            Box((0.092, 0.292, 0.086)),
            origin=Origin(xyz=(0.500, y * 1.55, -0.055)),
            material=powder_black,
            name=f"secondary_yoke_{'neg' if y < 0 else 'pos'}",
        )
    arm.visual(
        Box((0.400, 0.660, 0.020)),
        origin=Origin(xyz=(0.260, 0.0, -0.087)),
        material=powder_black,
        name="center_drain_rib",
    )
    arm.visual(
        Cylinder(radius=0.015, length=1.18),
        origin=Origin(xyz=(0.500, 0.0, -0.055), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=stainless,
        name="secondary_pin",
    )
    for y in (-0.61, 0.61):
        arm.visual(
            Cylinder(radius=0.040, length=0.016),
            origin=Origin(xyz=(0.500, math.copysign(0.598, y), -0.055), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=stainless,
            name=f"secondary_cap_{'neg' if y < 0 else 'pos'}",
        )
    for y in (-0.665, 0.665):
        arm.visual(
            Cylinder(radius=0.035, length=0.012),
            origin=Origin(xyz=(0.0, y, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=stainless,
            name=f"primary_cap_{'neg' if y < 0 else 'pos'}",
        )

    panel = model.part("visor_panel")
    panel.visual(
        Cylinder(radius=0.032, length=0.62),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=powder_black,
        name="secondary_barrel",
    )
    panel.visual(
        Box((0.050, 0.58, 0.030)),
        origin=Origin(xyz=(0.035, 0.0, -0.035)),
        material=powder_black,
        name="hinge_web",
    )
    panel.visual(
        mesh_from_geometry(_visor_panel_mesh(), "visor_panel_shell"),
        origin=Origin(),
        material=smoked_poly,
        name="panel_shell",
    )
    panel.visual(
        Box((0.040, 1.24, 0.054)),
        origin=Origin(xyz=(0.792, 0.0, -0.133)),
        material=sealant,
        name="front_drip_lip",
    )
    for y in (-0.625, 0.625):
        panel.visual(
            Box((0.660, 0.036, 0.034)),
            origin=Origin(xyz=(0.430, y, -0.066), rpy=(0.0, 0.105, 0.0)),
            material=sealant,
            name=f"side_gutter_{'neg' if y < 0 else 'pos'}",
        )
    panel.visual(
        Box((0.110, 1.18, 0.016)),
        origin=Origin(xyz=(0.103, 0.0, -0.031)),
        material=dark_gasket,
        name="rear_seal_flap",
    )

    model.articulation(
        "primary_pivot",
        ArticulationType.REVOLUTE,
        parent=roof,
        child=arm,
        origin=Origin(),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=90.0, velocity=0.7, lower=-0.30, upper=0.55),
    )
    model.articulation(
        "secondary_pivot",
        ArticulationType.REVOLUTE,
        parent=arm,
        child=panel,
        origin=Origin(xyz=(0.500, 0.0, -0.055)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=55.0, velocity=0.8, lower=-0.35, upper=0.55),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    roof = object_model.get_part("roof_header")
    arm = object_model.get_part("hinge_arm")
    panel = object_model.get_part("visor_panel")
    primary = object_model.get_articulation("primary_pivot")
    secondary = object_model.get_articulation("secondary_pivot")

    for knuckle in ("primary_knuckle_0", "primary_knuckle_1", "primary_knuckle_2"):
        ctx.allow_overlap(
            roof,
            arm,
            elem_a=knuckle,
            elem_b="primary_pin",
            reason="The stainless primary hinge pin is intentionally captured inside the sealed roof-hinge knuckle.",
        )
        ctx.expect_within(
            arm,
            roof,
            axes="xz",
            inner_elem="primary_pin",
            outer_elem=knuckle,
            margin=0.0,
            name=f"{knuckle} radially captures primary pin",
        )
        ctx.expect_overlap(
            roof,
            arm,
            axes="y",
            elem_a=knuckle,
            elem_b="primary_pin",
            min_overlap=0.18,
            name=f"{knuckle} has retained hinge-pin engagement",
        )

    ctx.allow_overlap(
        arm,
        panel,
        elem_a="secondary_pin",
        elem_b="secondary_barrel",
        reason="The secondary swing-pivot pin is intentionally captured inside the visor barrel bushing.",
    )
    ctx.expect_within(
        arm,
        panel,
        axes="xz",
        inner_elem="secondary_pin",
        outer_elem="secondary_barrel",
        margin=0.0,
        name="secondary pin is captured in barrel",
    )
    ctx.expect_overlap(
        arm,
        panel,
        axes="y",
        elem_a="secondary_pin",
        elem_b="secondary_barrel",
        min_overlap=0.58,
        name="secondary barrel retains pin engagement",
    )

    ctx.expect_gap(
        panel,
        roof,
        axis="x",
        min_gap=0.45,
        positive_elem="panel_shell",
        negative_elem="roof_plate",
        name="visor projects forward from roof header",
    )

    rest_panel = ctx.part_element_world_aabb(panel, elem="front_drip_lip")
    with ctx.pose({secondary: 0.45}):
        swung_panel = ctx.part_element_world_aabb(panel, elem="front_drip_lip")
    ctx.check(
        "secondary pivot lowers front drip lip",
        rest_panel is not None
        and swung_panel is not None
        and swung_panel[0][2] < rest_panel[0][2] - 0.05,
        details=f"rest={rest_panel}, swung={swung_panel}",
    )

    rest_origin = ctx.part_world_position(panel)
    with ctx.pose({primary: 0.45}):
        deployed_origin = ctx.part_world_position(panel)
    ctx.check(
        "primary roof pivot swings visor arm downward",
        rest_origin is not None
        and deployed_origin is not None
        and deployed_origin[2] < rest_origin[2] - 0.15,
        details=f"rest={rest_origin}, deployed={deployed_origin}",
    )

    return ctx.report()


object_model = build_object_model()
