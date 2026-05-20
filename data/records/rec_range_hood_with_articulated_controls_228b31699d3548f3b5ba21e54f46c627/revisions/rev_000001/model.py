from __future__ import annotations

from math import isclose, pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    KnobSkirt,
    Material,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


FRONT_FACE_X = 0.300
KNOB_CENTERS = {
    "upper_knob": (0.000, 0.136),
    "lower_knob": (0.000, 0.054),
    "left_knob": (-0.072, 0.095),
    "right_knob": (0.072, 0.095),
}


def _frustum_mesh() -> MeshGeometry:
    """Closed trapezoidal canopy below the chimney cover."""
    geom = MeshGeometry()
    vertices = [
        (0.275, -0.470, 0.145),
        (0.275, 0.470, 0.145),
        (-0.255, 0.470, 0.145),
        (-0.255, -0.470, 0.145),
        (0.100, -0.190, 0.325),
        (0.100, 0.190, 0.325),
        (-0.170, 0.190, 0.325),
        (-0.170, -0.190, 0.325),
    ]
    for x, y, z in vertices:
        geom.add_vertex(x, y, z)

    def quad(a: int, b: int, c: int, d: int) -> None:
        geom.add_face(a, b, c)
        geom.add_face(a, c, d)

    quad(0, 1, 5, 4)  # crisp sloped front face above the valance
    quad(1, 2, 6, 5)
    quad(2, 3, 7, 6)
    quad(3, 0, 4, 7)
    quad(3, 2, 1, 0)
    quad(4, 5, 6, 7)
    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="chimney_range_hood")

    stainless = Material("brushed_stainless", rgba=(0.72, 0.73, 0.70, 1.0))
    light_stainless = Material("light_stainless", rgba=(0.84, 0.84, 0.80, 1.0))
    shadow = Material("dark_filter", rgba=(0.045, 0.048, 0.050, 1.0))
    seam = Material("shadow_seam", rgba=(0.12, 0.12, 0.12, 1.0))
    knob_metal = Material("satin_knob_metal", rgba=(0.60, 0.61, 0.59, 1.0))
    indicator = Material("black_knob_index", rgba=(0.01, 0.01, 0.01, 1.0))

    body = model.part("body")
    body.visual(
        Box((0.040, 0.960, 0.180)),
        origin=Origin(xyz=(0.280, 0.0, 0.090)),
        material=stainless,
        name="front_valance",
    )
    body.visual(
        mesh_from_geometry(_frustum_mesh(), "sloped_canopy"),
        material=stainless,
        name="sloped_canopy",
    )
    body.visual(
        Box((0.040, 0.940, 0.055)),
        origin=Origin(xyz=(0.270, 0.0, 0.158)),
        material=stainless,
        name="valance_return",
    )
    body.visual(
        Box((0.200, 0.300, 0.730)),
        origin=Origin(xyz=(-0.035, 0.0, 0.685)),
        material=light_stainless,
        name="chimney_cover",
    )
    body.visual(
        Box((0.250, 0.360, 0.040)),
        origin=Origin(xyz=(-0.035, 0.0, 0.330)),
        material=light_stainless,
        name="chimney_base_flange",
    )
    body.visual(
        Box((0.540, 0.960, 0.024)),
        origin=Origin(xyz=(0.005, 0.0, 0.012)),
        material=stainless,
        name="bottom_lip",
    )
    for index, y in enumerate((-0.235, 0.235)):
        body.visual(
            Box((0.360, 0.390, 0.006)),
            origin=Origin(xyz=(0.005, y, -0.002)),
            material=shadow,
            name=f"filter_panel_{index}",
        )
        for slat in range(5):
            body.visual(
                Box((0.325, 0.010, 0.007)),
                origin=Origin(xyz=(0.005, y - 0.120 + slat * 0.060, 0.002)),
                material=seam,
                name=f"filter_slat_{index}_{slat}",
            )
    body.visual(
        Box((0.018, 0.970, 0.020)),
        origin=Origin(xyz=(0.280, 0.0, 0.181)),
        material=light_stainless,
        name="valance_top_crease",
    )
    body.visual(
        Box((0.002, 0.006, 0.660)),
        origin=Origin(xyz=(0.0645, -0.149, 0.695)),
        material=seam,
        name="chimney_left_fold",
    )
    body.visual(
        Box((0.002, 0.006, 0.660)),
        origin=Origin(xyz=(0.0645, 0.149, 0.695)),
        material=seam,
        name="chimney_right_fold",
    )

    knob_mesh = mesh_from_geometry(
        KnobGeometry(
            0.055,
            0.028,
            body_style="skirted",
            top_diameter=0.042,
            edge_radius=0.001,
            skirt=KnobSkirt(0.066, 0.007, flare=0.05, chamfer=0.0012),
            grip=KnobGrip(style="fluted", count=20, depth=0.0012),
            indicator=KnobIndicator(style="line", mode="engraved", depth=0.0007),
            center=False,
        ),
        "rotary_knob",
    )

    for knob_name, (y, z) in KNOB_CENTERS.items():
        knob = model.part(knob_name)
        knob.visual(
            knob_mesh,
            origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
            material=knob_metal,
            name="knob_shell",
        )
        knob.visual(
            Box((0.003, 0.005, 0.023)),
            origin=Origin(xyz=(0.0345, 0.0, 0.007)),
            material=indicator,
            name="index_mark",
        )
        model.articulation(
            f"body_to_{knob_name}",
            ArticulationType.CONTINUOUS,
            parent=body,
            child=knob,
            origin=Origin(xyz=(FRONT_FACE_X, y, z)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=0.35, velocity=8.0),
        )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    knob_names = tuple(KNOB_CENTERS.keys())
    joints = [object_model.get_articulation(f"body_to_{name}") for name in knob_names]

    ctx.check(
        "only four knob articulations",
        len(object_model.articulations) == 4,
        details=f"articulations={[joint.name for joint in object_model.articulations]}",
    )
    for joint in joints:
        ctx.check(
            f"{joint.name} is continuous",
            joint.articulation_type == ArticulationType.CONTINUOUS,
            details=f"type={joint.articulation_type}",
        )
        ctx.check(
            f"{joint.name} axis is front-to-back",
            tuple(round(v, 6) for v in joint.axis) == (1.0, 0.0, 0.0),
            details=f"axis={joint.axis}",
        )

    for knob_name in knob_names:
        knob = object_model.get_part(knob_name)
        ctx.expect_gap(
            knob,
            body,
            axis="x",
            positive_elem="knob_shell",
            negative_elem="front_valance",
            min_gap=-0.0002,
            max_gap=0.002,
            name=f"{knob_name} seats on front valance",
        )
        ctx.expect_overlap(
            knob,
            body,
            axes="yz",
            elem_a="knob_shell",
            elem_b="front_valance",
            min_overlap=0.045,
            name=f"{knob_name} is centered on the valance face",
        )

    positions = {name: ctx.part_world_position(object_model.get_part(name)) for name in knob_names}
    ctx.check(
        "knobs form a diamond on the front center",
        all(pos is not None for pos in positions.values())
        and isclose(positions["upper_knob"][1], positions["lower_knob"][1], abs_tol=1e-6)
        and isclose(positions["left_knob"][2], positions["right_knob"][2], abs_tol=1e-6)
        and positions["upper_knob"][2] > positions["left_knob"][2] > positions["lower_knob"][2]
        and positions["left_knob"][1] < positions["upper_knob"][1] < positions["right_knob"][1],
        details=f"positions={positions}",
    )

    return ctx.report()


object_model = build_object_model()
